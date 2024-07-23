// Benchmark "adder" written by ABC on Thu Jul 18 01:33:03 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n341, new_n342, new_n344, new_n346, new_n348,
    new_n349, new_n351, new_n352, new_n354;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nanb02aa1n02x5               g002(.a(\b[9] ), .b(new_n97), .out0(new_n98));
  nand02aa1n04x5               g003(.a(\b[9] ), .b(\a[10] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand22aa1n03x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  tech160nm_fiaoi012aa1n05x5   g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  nor002aa1n04x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nand02aa1d04x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  norb02aa1n03x5               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  norp02aa1n04x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand42aa1n02x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  norb02aa1n03x5               g014(.a(new_n109), .b(new_n108), .out0(new_n110));
  nanb03aa1n03x5               g015(.a(new_n104), .b(new_n110), .c(new_n107), .out0(new_n111));
  aoi012aa1n02x7               g016(.a(new_n105), .b(new_n108), .c(new_n106), .o1(new_n112));
  nor042aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nand02aa1n04x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nor002aa1n03x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nand42aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nano23aa1n02x4               g021(.a(new_n113), .b(new_n115), .c(new_n116), .d(new_n114), .out0(new_n117));
  xorc02aa1n12x5               g022(.a(\a[8] ), .b(\b[7] ), .out0(new_n118));
  nor042aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  and002aa1n12x5               g024(.a(\b[6] ), .b(\a[7] ), .o(new_n120));
  nor002aa1n04x5               g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  nand03aa1n02x5               g026(.a(new_n117), .b(new_n118), .c(new_n121), .o1(new_n122));
  tech160nm_fiaoi012aa1n05x5   g027(.a(new_n113), .b(new_n115), .c(new_n114), .o1(new_n123));
  inv020aa1n02x5               g028(.a(new_n123), .o1(new_n124));
  inv000aa1d42x5               g029(.a(\a[8] ), .o1(new_n125));
  inv000aa1d42x5               g030(.a(\b[7] ), .o1(new_n126));
  oaoi03aa1n03x5               g031(.a(new_n125), .b(new_n126), .c(new_n119), .o1(new_n127));
  inv000aa1n02x5               g032(.a(new_n127), .o1(new_n128));
  aoi013aa1n03x5               g033(.a(new_n128), .b(new_n124), .c(new_n121), .d(new_n118), .o1(new_n129));
  aoai13aa1n04x5               g034(.a(new_n129), .b(new_n122), .c(new_n111), .d(new_n112), .o1(new_n130));
  xnrc02aa1n12x5               g035(.a(\b[8] ), .b(\a[9] ), .out0(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  aoi012aa1n02x5               g037(.a(new_n100), .b(new_n130), .c(new_n132), .o1(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n98), .c(new_n99), .out0(\s[10] ));
  nona23aa1n06x5               g039(.a(new_n109), .b(new_n106), .c(new_n105), .d(new_n108), .out0(new_n135));
  oai012aa1n06x5               g040(.a(new_n112), .b(new_n135), .c(new_n104), .o1(new_n136));
  nona23aa1n02x4               g041(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n137));
  nano22aa1n03x7               g042(.a(new_n137), .b(new_n118), .c(new_n121), .out0(new_n138));
  xnrc02aa1n02x5               g043(.a(\b[7] ), .b(\a[8] ), .out0(new_n139));
  inv020aa1n02x5               g044(.a(new_n121), .o1(new_n140));
  oai013aa1n03x5               g045(.a(new_n127), .b(new_n140), .c(new_n139), .d(new_n123), .o1(new_n141));
  nano22aa1n06x5               g046(.a(new_n131), .b(new_n98), .c(new_n99), .out0(new_n142));
  aoai13aa1n02x5               g047(.a(new_n142), .b(new_n141), .c(new_n136), .d(new_n138), .o1(new_n143));
  oaih22aa1d12x5               g048(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n144));
  nanp02aa1n02x5               g049(.a(new_n144), .b(new_n99), .o1(new_n145));
  norp02aa1n04x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  nand22aa1n03x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  xnbna2aa1n03x5               g053(.a(new_n148), .b(new_n143), .c(new_n145), .out0(\s[11] ));
  inv020aa1n03x5               g054(.a(new_n146), .o1(new_n150));
  aob012aa1n02x5               g055(.a(new_n148), .b(new_n143), .c(new_n145), .out0(new_n151));
  nor002aa1n03x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  nand02aa1n06x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(new_n154));
  xobna2aa1n03x5               g059(.a(new_n154), .b(new_n151), .c(new_n150), .out0(\s[12] ));
  nano23aa1n06x5               g060(.a(new_n146), .b(new_n152), .c(new_n153), .d(new_n147), .out0(new_n156));
  and002aa1n02x5               g061(.a(new_n142), .b(new_n156), .o(new_n157));
  aoai13aa1n02x5               g062(.a(new_n157), .b(new_n141), .c(new_n136), .d(new_n138), .o1(new_n158));
  inv000aa1n02x5               g063(.a(new_n152), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n153), .o1(new_n160));
  nand23aa1n03x5               g065(.a(new_n144), .b(new_n99), .c(new_n147), .o1(new_n161));
  aoai13aa1n12x5               g066(.a(new_n159), .b(new_n160), .c(new_n161), .d(new_n150), .o1(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(new_n158), .b(new_n163), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  nand42aa1d28x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  aoi012aa1n02x5               g072(.a(new_n166), .b(new_n164), .c(new_n167), .o1(new_n168));
  xnrb03aa1n02x5               g073(.a(new_n168), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nand42aa1n10x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nano23aa1n06x5               g076(.a(new_n166), .b(new_n170), .c(new_n171), .d(new_n167), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n162), .c(new_n130), .d(new_n157), .o1(new_n173));
  oa0012aa1n02x5               g078(.a(new_n171), .b(new_n170), .c(new_n166), .o(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  nor002aa1n02x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nanp02aa1n04x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  norb02aa1n06x4               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  xnbna2aa1n03x5               g083(.a(new_n178), .b(new_n173), .c(new_n175), .out0(\s[15] ));
  nanp02aa1n02x5               g084(.a(new_n173), .b(new_n175), .o1(new_n180));
  xorc02aa1n12x5               g085(.a(\a[16] ), .b(\b[15] ), .out0(new_n181));
  inv000aa1d42x5               g086(.a(new_n181), .o1(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n176), .c(new_n180), .d(new_n177), .o1(new_n183));
  aoai13aa1n02x5               g088(.a(new_n178), .b(new_n174), .c(new_n164), .d(new_n172), .o1(new_n184));
  nona22aa1n02x4               g089(.a(new_n184), .b(new_n182), .c(new_n176), .out0(new_n185));
  nanp02aa1n02x5               g090(.a(new_n183), .b(new_n185), .o1(\s[16] ));
  nand23aa1n04x5               g091(.a(new_n172), .b(new_n178), .c(new_n181), .o1(new_n187));
  nano22aa1n03x7               g092(.a(new_n187), .b(new_n142), .c(new_n156), .out0(new_n188));
  aoai13aa1n06x5               g093(.a(new_n188), .b(new_n141), .c(new_n136), .d(new_n138), .o1(new_n189));
  inv030aa1n02x5               g094(.a(new_n187), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\a[16] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(\b[15] ), .o1(new_n192));
  oai112aa1n02x5               g097(.a(new_n171), .b(new_n177), .c(new_n170), .d(new_n166), .o1(new_n193));
  tech160nm_fioai012aa1n03p5x5 g098(.a(new_n193), .b(\b[14] ), .c(\a[15] ), .o1(new_n194));
  oaib12aa1n02x5               g099(.a(new_n194), .b(new_n192), .c(\a[16] ), .out0(new_n195));
  oaib12aa1n02x5               g100(.a(new_n195), .b(\b[15] ), .c(new_n191), .out0(new_n196));
  aoi012aa1d18x5               g101(.a(new_n196), .b(new_n162), .c(new_n190), .o1(new_n197));
  nand02aa1d08x5               g102(.a(new_n189), .b(new_n197), .o1(new_n198));
  nor042aa1n12x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  nand42aa1n06x5               g104(.a(\b[16] ), .b(\a[17] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  aoi112aa1n02x5               g106(.a(new_n196), .b(new_n201), .c(new_n190), .d(new_n162), .o1(new_n202));
  aoi022aa1n02x5               g107(.a(new_n198), .b(new_n201), .c(new_n189), .d(new_n202), .o1(\s[17] ));
  tech160nm_fiaoi012aa1n05x5   g108(.a(new_n199), .b(new_n198), .c(new_n201), .o1(new_n204));
  xnrb03aa1n03x5               g109(.a(new_n204), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nanp02aa1n02x5               g110(.a(new_n161), .b(new_n150), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(new_n206), .b(new_n153), .o1(new_n207));
  oaoi03aa1n02x5               g112(.a(new_n191), .b(new_n192), .c(new_n194), .o1(new_n208));
  aoai13aa1n04x5               g113(.a(new_n208), .b(new_n187), .c(new_n207), .d(new_n159), .o1(new_n209));
  nor042aa1n09x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  nanp02aa1n12x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  nano23aa1d12x5               g116(.a(new_n199), .b(new_n210), .c(new_n211), .d(new_n200), .out0(new_n212));
  aoai13aa1n02x5               g117(.a(new_n212), .b(new_n209), .c(new_n130), .d(new_n188), .o1(new_n213));
  oa0012aa1n02x5               g118(.a(new_n211), .b(new_n210), .c(new_n199), .o(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  nor042aa1n06x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  nand22aa1n09x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  norb02aa1n03x5               g122(.a(new_n217), .b(new_n216), .out0(new_n218));
  xnbna2aa1n03x5               g123(.a(new_n218), .b(new_n213), .c(new_n215), .out0(\s[19] ));
  xnrc02aa1n02x5               g124(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n02x5               g125(.a(new_n213), .b(new_n215), .o1(new_n221));
  nor042aa1n04x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  nand22aa1n12x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  nanb02aa1n03x5               g128(.a(new_n222), .b(new_n223), .out0(new_n224));
  aoai13aa1n02x5               g129(.a(new_n224), .b(new_n216), .c(new_n221), .d(new_n217), .o1(new_n225));
  aoai13aa1n06x5               g130(.a(new_n218), .b(new_n214), .c(new_n198), .d(new_n212), .o1(new_n226));
  nona22aa1n03x5               g131(.a(new_n226), .b(new_n224), .c(new_n216), .out0(new_n227));
  nanp02aa1n03x5               g132(.a(new_n225), .b(new_n227), .o1(\s[20] ));
  nanb03aa1d24x5               g133(.a(new_n224), .b(new_n212), .c(new_n218), .out0(new_n229));
  nanb03aa1n06x5               g134(.a(new_n222), .b(new_n223), .c(new_n217), .out0(new_n230));
  orn002aa1n02x5               g135(.a(\a[19] ), .b(\b[18] ), .o(new_n231));
  oai112aa1n06x5               g136(.a(new_n231), .b(new_n211), .c(new_n210), .d(new_n199), .o1(new_n232));
  aoi012aa1n09x5               g137(.a(new_n222), .b(new_n216), .c(new_n223), .o1(new_n233));
  oai012aa1d24x5               g138(.a(new_n233), .b(new_n232), .c(new_n230), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoai13aa1n04x5               g140(.a(new_n235), .b(new_n229), .c(new_n189), .d(new_n197), .o1(new_n236));
  nor042aa1n06x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  nanp02aa1n04x5               g142(.a(\b[20] ), .b(\a[21] ), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n238), .b(new_n237), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n229), .o1(new_n240));
  aoi112aa1n02x5               g145(.a(new_n239), .b(new_n234), .c(new_n198), .d(new_n240), .o1(new_n241));
  aoi012aa1n02x5               g146(.a(new_n241), .b(new_n236), .c(new_n239), .o1(\s[21] ));
  nor042aa1n04x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  nand42aa1n08x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n237), .c(new_n236), .d(new_n239), .o1(new_n247));
  nand42aa1n03x5               g152(.a(new_n236), .b(new_n239), .o1(new_n248));
  nona22aa1n03x5               g153(.a(new_n248), .b(new_n246), .c(new_n237), .out0(new_n249));
  nanp02aa1n03x5               g154(.a(new_n249), .b(new_n247), .o1(\s[22] ));
  nano23aa1d15x5               g155(.a(new_n237), .b(new_n243), .c(new_n244), .d(new_n238), .out0(new_n251));
  nanb02aa1n02x5               g156(.a(new_n229), .b(new_n251), .out0(new_n252));
  aoi012aa1n02x5               g157(.a(new_n252), .b(new_n189), .c(new_n197), .o1(new_n253));
  aoi012aa1d18x5               g158(.a(new_n243), .b(new_n237), .c(new_n244), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  aoi012aa1n02x5               g160(.a(new_n255), .b(new_n234), .c(new_n251), .o1(new_n256));
  aoai13aa1n04x5               g161(.a(new_n256), .b(new_n252), .c(new_n189), .d(new_n197), .o1(new_n257));
  xorc02aa1n12x5               g162(.a(\a[23] ), .b(\b[22] ), .out0(new_n258));
  aoi112aa1n02x5               g163(.a(new_n258), .b(new_n255), .c(new_n234), .d(new_n251), .o1(new_n259));
  aboi22aa1n03x5               g164(.a(new_n253), .b(new_n259), .c(new_n257), .d(new_n258), .out0(\s[23] ));
  norp02aa1n02x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  tech160nm_fixnrc02aa1n05x5   g166(.a(\b[23] ), .b(\a[24] ), .out0(new_n262));
  aoai13aa1n04x5               g167(.a(new_n262), .b(new_n261), .c(new_n257), .d(new_n258), .o1(new_n263));
  tech160nm_finand02aa1n03p5x5 g168(.a(new_n257), .b(new_n258), .o1(new_n264));
  nona22aa1n02x4               g169(.a(new_n264), .b(new_n262), .c(new_n261), .out0(new_n265));
  nanp02aa1n02x5               g170(.a(new_n265), .b(new_n263), .o1(\s[24] ));
  norb02aa1n06x4               g171(.a(new_n258), .b(new_n262), .out0(new_n267));
  nano22aa1n03x7               g172(.a(new_n229), .b(new_n267), .c(new_n251), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n209), .c(new_n130), .d(new_n188), .o1(new_n269));
  nano22aa1n03x5               g174(.a(new_n222), .b(new_n217), .c(new_n223), .out0(new_n270));
  oai012aa1n02x5               g175(.a(new_n211), .b(\b[18] ), .c(\a[19] ), .o1(new_n271));
  oab012aa1n03x5               g176(.a(new_n271), .b(new_n199), .c(new_n210), .out0(new_n272));
  inv020aa1n02x5               g177(.a(new_n233), .o1(new_n273));
  aoai13aa1n06x5               g178(.a(new_n251), .b(new_n273), .c(new_n272), .d(new_n270), .o1(new_n274));
  inv000aa1n02x5               g179(.a(new_n267), .o1(new_n275));
  orn002aa1n02x5               g180(.a(\a[23] ), .b(\b[22] ), .o(new_n276));
  oao003aa1n02x5               g181(.a(\a[24] ), .b(\b[23] ), .c(new_n276), .carry(new_n277));
  aoai13aa1n12x5               g182(.a(new_n277), .b(new_n275), .c(new_n274), .d(new_n254), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  nanp02aa1n02x5               g184(.a(new_n269), .b(new_n279), .o1(new_n280));
  xorc02aa1n12x5               g185(.a(\a[25] ), .b(\b[24] ), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n267), .b(new_n255), .c(new_n234), .d(new_n251), .o1(new_n282));
  nano22aa1n02x4               g187(.a(new_n281), .b(new_n282), .c(new_n277), .out0(new_n283));
  aoi022aa1n02x5               g188(.a(new_n280), .b(new_n281), .c(new_n269), .d(new_n283), .o1(\s[25] ));
  norp02aa1n02x5               g189(.a(\b[24] ), .b(\a[25] ), .o1(new_n285));
  xorc02aa1n12x5               g190(.a(\a[26] ), .b(\b[25] ), .out0(new_n286));
  inv000aa1d42x5               g191(.a(new_n286), .o1(new_n287));
  aoai13aa1n02x5               g192(.a(new_n287), .b(new_n285), .c(new_n280), .d(new_n281), .o1(new_n288));
  aoai13aa1n03x5               g193(.a(new_n281), .b(new_n278), .c(new_n198), .d(new_n268), .o1(new_n289));
  nona22aa1n03x5               g194(.a(new_n289), .b(new_n287), .c(new_n285), .out0(new_n290));
  nanp02aa1n03x5               g195(.a(new_n288), .b(new_n290), .o1(\s[26] ));
  and002aa1n24x5               g196(.a(new_n286), .b(new_n281), .o(new_n292));
  nano32aa1n03x7               g197(.a(new_n229), .b(new_n292), .c(new_n251), .d(new_n267), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n209), .c(new_n130), .d(new_n188), .o1(new_n294));
  inv000aa1d42x5               g199(.a(\a[26] ), .o1(new_n295));
  inv000aa1d42x5               g200(.a(\b[25] ), .o1(new_n296));
  oaoi03aa1n02x5               g201(.a(new_n295), .b(new_n296), .c(new_n285), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  aoi012aa1n09x5               g203(.a(new_n298), .b(new_n278), .c(new_n292), .o1(new_n299));
  nanp02aa1n06x5               g204(.a(new_n299), .b(new_n294), .o1(new_n300));
  xorc02aa1n12x5               g205(.a(\a[27] ), .b(\b[26] ), .out0(new_n301));
  aoi112aa1n02x5               g206(.a(new_n301), .b(new_n298), .c(new_n278), .d(new_n292), .o1(new_n302));
  aoi022aa1n02x5               g207(.a(new_n300), .b(new_n301), .c(new_n294), .d(new_n302), .o1(\s[27] ));
  norp02aa1n02x5               g208(.a(\b[26] ), .b(\a[27] ), .o1(new_n304));
  norp02aa1n02x5               g209(.a(\b[27] ), .b(\a[28] ), .o1(new_n305));
  nanp02aa1n02x5               g210(.a(\b[27] ), .b(\a[28] ), .o1(new_n306));
  nanb02aa1n02x5               g211(.a(new_n305), .b(new_n306), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n304), .c(new_n300), .d(new_n301), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n292), .o1(new_n309));
  aoai13aa1n06x5               g214(.a(new_n297), .b(new_n309), .c(new_n282), .d(new_n277), .o1(new_n310));
  aoai13aa1n03x5               g215(.a(new_n301), .b(new_n310), .c(new_n198), .d(new_n293), .o1(new_n311));
  nona22aa1n03x5               g216(.a(new_n311), .b(new_n307), .c(new_n304), .out0(new_n312));
  nanp02aa1n03x5               g217(.a(new_n308), .b(new_n312), .o1(\s[28] ));
  norb02aa1d21x5               g218(.a(new_n301), .b(new_n307), .out0(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n310), .c(new_n198), .d(new_n293), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[29] ), .b(\b[28] ), .out0(new_n316));
  oai012aa1n02x5               g221(.a(new_n306), .b(new_n305), .c(new_n304), .o1(new_n317));
  norb02aa1n02x5               g222(.a(new_n317), .b(new_n316), .out0(new_n318));
  inv000aa1d42x5               g223(.a(new_n314), .o1(new_n319));
  aoai13aa1n02x5               g224(.a(new_n317), .b(new_n319), .c(new_n299), .d(new_n294), .o1(new_n320));
  aoi022aa1n03x5               g225(.a(new_n320), .b(new_n316), .c(new_n315), .d(new_n318), .o1(\s[29] ));
  xorb03aa1n02x5               g226(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1d18x5               g227(.a(new_n307), .b(new_n301), .c(new_n316), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n310), .c(new_n198), .d(new_n293), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[30] ), .b(\b[29] ), .out0(new_n325));
  oao003aa1n02x5               g230(.a(\a[29] ), .b(\b[28] ), .c(new_n317), .carry(new_n326));
  norb02aa1n02x5               g231(.a(new_n326), .b(new_n325), .out0(new_n327));
  inv000aa1d42x5               g232(.a(new_n323), .o1(new_n328));
  aoai13aa1n02x5               g233(.a(new_n326), .b(new_n328), .c(new_n299), .d(new_n294), .o1(new_n329));
  aoi022aa1n03x5               g234(.a(new_n329), .b(new_n325), .c(new_n324), .d(new_n327), .o1(\s[30] ));
  nanp03aa1n02x5               g235(.a(new_n314), .b(new_n316), .c(new_n325), .o1(new_n331));
  nanb02aa1n02x5               g236(.a(new_n331), .b(new_n300), .out0(new_n332));
  xorc02aa1n02x5               g237(.a(\a[31] ), .b(\b[30] ), .out0(new_n333));
  and002aa1n02x5               g238(.a(\b[29] ), .b(\a[30] ), .o(new_n334));
  oabi12aa1n02x5               g239(.a(new_n333), .b(\a[30] ), .c(\b[29] ), .out0(new_n335));
  oab012aa1n02x4               g240(.a(new_n335), .b(new_n326), .c(new_n334), .out0(new_n336));
  oao003aa1n02x5               g241(.a(\a[30] ), .b(\b[29] ), .c(new_n326), .carry(new_n337));
  aoai13aa1n03x5               g242(.a(new_n337), .b(new_n331), .c(new_n299), .d(new_n294), .o1(new_n338));
  aoi022aa1n02x5               g243(.a(new_n332), .b(new_n336), .c(new_n338), .d(new_n333), .o1(\s[31] ));
  xnrb03aa1n02x5               g244(.a(new_n104), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoai13aa1n02x5               g245(.a(new_n110), .b(new_n101), .c(new_n103), .d(new_n102), .o1(new_n341));
  aoib12aa1n02x5               g246(.a(new_n108), .b(new_n106), .c(new_n105), .out0(new_n342));
  aboi22aa1n03x5               g247(.a(new_n105), .b(new_n136), .c(new_n341), .d(new_n342), .out0(\s[4] ));
  norb02aa1n02x5               g248(.a(new_n116), .b(new_n115), .out0(new_n344));
  xnbna2aa1n03x5               g249(.a(new_n344), .b(new_n111), .c(new_n112), .out0(\s[5] ));
  tech160nm_fiao0012aa1n02p5x5 g250(.a(new_n115), .b(new_n136), .c(new_n116), .o(new_n346));
  xorb03aa1n02x5               g251(.a(new_n346), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g252(.a(\a[7] ), .o1(new_n348));
  aoi012aa1n02x5               g253(.a(new_n113), .b(new_n346), .c(new_n114), .o1(new_n349));
  xorb03aa1n02x5               g254(.a(new_n349), .b(\b[6] ), .c(new_n348), .out0(\s[7] ));
  aoi112aa1n02x5               g255(.a(new_n118), .b(new_n120), .c(new_n349), .d(new_n121), .o1(new_n351));
  aoai13aa1n02x5               g256(.a(new_n118), .b(new_n120), .c(new_n349), .d(new_n121), .o1(new_n352));
  nanb02aa1n02x5               g257(.a(new_n351), .b(new_n352), .out0(\s[8] ));
  aoi112aa1n02x5               g258(.a(new_n141), .b(new_n132), .c(new_n136), .d(new_n138), .o1(new_n354));
  aoi012aa1n02x5               g259(.a(new_n354), .b(new_n130), .c(new_n132), .o1(\s[9] ));
endmodule


