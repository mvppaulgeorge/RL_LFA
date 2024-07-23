// Benchmark "adder" written by ABC on Wed Jul 17 19:04:47 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n335, new_n336, new_n339, new_n340,
    new_n342, new_n343, new_n344, new_n345, new_n346, new_n348;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  oai022aa1n06x5               g003(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n99));
  nor042aa1n06x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nand42aa1n03x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  norb02aa1n06x5               g006(.a(new_n101), .b(new_n100), .out0(new_n102));
  aoi022aa1n12x5               g007(.a(\b[7] ), .b(\a[8] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n103));
  inv020aa1n02x5               g008(.a(new_n100), .o1(new_n104));
  oaoi03aa1n02x5               g009(.a(\a[8] ), .b(\b[7] ), .c(new_n104), .o1(new_n105));
  aoi013aa1n09x5               g010(.a(new_n105), .b(new_n102), .c(new_n103), .d(new_n99), .o1(new_n106));
  and002aa1n02x5               g011(.a(\b[4] ), .b(\a[5] ), .o(new_n107));
  nano32aa1n03x7               g012(.a(new_n107), .b(new_n103), .c(new_n104), .d(new_n101), .out0(new_n108));
  and002aa1n02x5               g013(.a(\b[0] ), .b(\a[1] ), .o(new_n109));
  oaoi03aa1n02x5               g014(.a(\a[2] ), .b(\b[1] ), .c(new_n109), .o1(new_n110));
  nor002aa1n12x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nand22aa1n04x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nor042aa1n03x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nano23aa1n06x5               g019(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n115));
  aoi012aa1n09x5               g020(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n116));
  inv030aa1n02x5               g021(.a(new_n116), .o1(new_n117));
  aoai13aa1n06x5               g022(.a(new_n108), .b(new_n117), .c(new_n115), .d(new_n110), .o1(new_n118));
  nand02aa1d08x5               g023(.a(new_n118), .b(new_n106), .o1(new_n119));
  oaoi03aa1n09x5               g024(.a(new_n97), .b(new_n98), .c(new_n119), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[10] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[9] ), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(new_n122), .b(new_n121), .o1(new_n123));
  nand02aa1d28x5               g028(.a(\b[9] ), .b(\a[10] ), .o1(new_n124));
  xnbna2aa1n03x5               g029(.a(new_n120), .b(new_n124), .c(new_n123), .out0(\s[10] ));
  nand02aa1d16x5               g030(.a(\b[10] ), .b(\a[11] ), .o1(new_n126));
  nor002aa1d32x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  norb02aa1n12x5               g032(.a(new_n126), .b(new_n127), .out0(new_n128));
  inv000aa1d42x5               g033(.a(new_n128), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(new_n120), .b(new_n123), .o1(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n129), .b(new_n130), .c(new_n124), .out0(\s[11] ));
  inv000aa1d42x5               g036(.a(new_n127), .o1(new_n132));
  inv000aa1d42x5               g037(.a(new_n124), .o1(new_n133));
  nona22aa1n02x4               g038(.a(new_n130), .b(new_n129), .c(new_n133), .out0(new_n134));
  norp02aa1n02x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nand02aa1d24x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aobi12aa1n02x5               g042(.a(new_n137), .b(new_n134), .c(new_n132), .out0(new_n138));
  aoi112aa1n02x5               g043(.a(new_n129), .b(new_n133), .c(new_n120), .d(new_n123), .o1(new_n139));
  norp03aa1n02x5               g044(.a(new_n139), .b(new_n137), .c(new_n127), .o1(new_n140));
  norp02aa1n02x5               g045(.a(new_n138), .b(new_n140), .o1(\s[12] ));
  oai022aa1d24x5               g046(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n142));
  nanb03aa1n06x5               g047(.a(new_n127), .b(new_n124), .c(new_n126), .out0(new_n143));
  nano22aa1n03x7               g048(.a(new_n143), .b(new_n136), .c(new_n142), .out0(new_n144));
  tech160nm_fiaoi012aa1n04x5   g049(.a(new_n135), .b(new_n127), .c(new_n136), .o1(new_n145));
  inv020aa1n02x5               g050(.a(new_n145), .o1(new_n146));
  norp02aa1n02x5               g051(.a(new_n144), .b(new_n146), .o1(new_n147));
  inv000aa1n02x5               g052(.a(new_n106), .o1(new_n148));
  inv000aa1d42x5               g053(.a(\a[1] ), .o1(new_n149));
  inv000aa1d42x5               g054(.a(\b[0] ), .o1(new_n150));
  norp02aa1n02x5               g055(.a(\b[1] ), .b(\a[2] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[1] ), .b(\a[2] ), .o1(new_n152));
  oaoi13aa1n04x5               g057(.a(new_n151), .b(new_n152), .c(new_n149), .d(new_n150), .o1(new_n153));
  nona23aa1n09x5               g058(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n154));
  oaih12aa1n12x5               g059(.a(new_n116), .b(new_n154), .c(new_n153), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[8] ), .b(\a[9] ), .o1(new_n156));
  nano22aa1n03x7               g061(.a(new_n143), .b(new_n156), .c(new_n136), .out0(new_n157));
  aoai13aa1n04x5               g062(.a(new_n157), .b(new_n148), .c(new_n155), .d(new_n108), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(new_n158), .b(new_n147), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n06x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  aoi012aa1n02x5               g067(.a(new_n161), .b(new_n159), .c(new_n162), .o1(new_n163));
  xnrb03aa1n02x5               g068(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n06x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nona23aa1n02x4               g071(.a(new_n166), .b(new_n162), .c(new_n161), .d(new_n165), .out0(new_n167));
  oa0012aa1n02x5               g072(.a(new_n166), .b(new_n165), .c(new_n161), .o(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  aoai13aa1n03x5               g074(.a(new_n169), .b(new_n167), .c(new_n158), .d(new_n147), .o1(new_n170));
  xorb03aa1n02x5               g075(.a(new_n170), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor022aa1n08x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nanp02aa1n03x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nor042aa1n04x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nanp02aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  norb02aa1n06x4               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  aoi112aa1n02x5               g081(.a(new_n172), .b(new_n176), .c(new_n170), .d(new_n173), .o1(new_n177));
  aoai13aa1n02x5               g082(.a(new_n176), .b(new_n172), .c(new_n170), .d(new_n173), .o1(new_n178));
  norb02aa1n02x7               g083(.a(new_n178), .b(new_n177), .out0(\s[16] ));
  tech160nm_fiaoi012aa1n04x5   g084(.a(new_n148), .b(new_n155), .c(new_n108), .o1(new_n180));
  norb02aa1n09x5               g085(.a(new_n173), .b(new_n172), .out0(new_n181));
  nano22aa1n03x7               g086(.a(new_n167), .b(new_n181), .c(new_n176), .out0(new_n182));
  inv000aa1d42x5               g087(.a(new_n142), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n136), .o1(new_n184));
  nona22aa1n02x4               g089(.a(new_n128), .b(new_n184), .c(new_n133), .out0(new_n185));
  nano32aa1n03x7               g090(.a(new_n185), .b(new_n145), .c(new_n183), .d(new_n156), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(new_n186), .b(new_n182), .o1(new_n187));
  inv000aa1d42x5               g092(.a(new_n174), .o1(new_n188));
  inv000aa1n02x5               g093(.a(new_n172), .o1(new_n189));
  oai112aa1n02x5               g094(.a(new_n166), .b(new_n173), .c(new_n165), .d(new_n161), .o1(new_n190));
  aob012aa1n03x5               g095(.a(new_n175), .b(new_n190), .c(new_n189), .out0(new_n191));
  nona23aa1n03x5               g096(.a(new_n128), .b(new_n142), .c(new_n184), .d(new_n133), .out0(new_n192));
  nano23aa1n03x7               g097(.a(new_n161), .b(new_n165), .c(new_n166), .d(new_n162), .out0(new_n193));
  nanp03aa1n02x5               g098(.a(new_n193), .b(new_n181), .c(new_n176), .o1(new_n194));
  aoi012aa1n02x5               g099(.a(new_n194), .b(new_n192), .c(new_n145), .o1(new_n195));
  nano22aa1n02x4               g100(.a(new_n195), .b(new_n191), .c(new_n188), .out0(new_n196));
  oai012aa1n06x5               g101(.a(new_n196), .b(new_n180), .c(new_n187), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nano32aa1n06x5               g103(.a(new_n194), .b(new_n192), .c(new_n157), .d(new_n145), .out0(new_n199));
  oaih12aa1n06x5               g104(.a(new_n182), .b(new_n144), .c(new_n146), .o1(new_n200));
  nand23aa1n09x5               g105(.a(new_n200), .b(new_n188), .c(new_n191), .o1(new_n201));
  aoi012aa1n06x5               g106(.a(new_n201), .b(new_n119), .c(new_n199), .o1(new_n202));
  oaoi03aa1n02x5               g107(.a(\a[17] ), .b(\b[16] ), .c(new_n202), .o1(new_n203));
  xorb03aa1n02x5               g108(.a(new_n203), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g109(.a(\a[17] ), .o1(new_n205));
  inv020aa1n04x5               g110(.a(\a[18] ), .o1(new_n206));
  xroi22aa1d06x4               g111(.a(new_n205), .b(\b[16] ), .c(new_n206), .d(\b[17] ), .out0(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n201), .c(new_n119), .d(new_n199), .o1(new_n208));
  inv000aa1d42x5               g113(.a(\b[17] ), .o1(new_n209));
  oai022aa1d24x5               g114(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n210));
  oaib12aa1n18x5               g115(.a(new_n210), .b(new_n209), .c(\a[18] ), .out0(new_n211));
  xorc02aa1n12x5               g116(.a(\a[19] ), .b(\b[18] ), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n208), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g119(.a(\a[19] ), .o1(new_n215));
  nanb02aa1n02x5               g120(.a(\b[18] ), .b(new_n215), .out0(new_n216));
  inv040aa1n03x5               g121(.a(new_n212), .o1(new_n217));
  tech160nm_fiaoi012aa1n02p5x5 g122(.a(new_n217), .b(new_n208), .c(new_n211), .o1(new_n218));
  xnrc02aa1n03x5               g123(.a(\b[19] ), .b(\a[20] ), .out0(new_n219));
  nano22aa1n02x4               g124(.a(new_n218), .b(new_n216), .c(new_n219), .out0(new_n220));
  aoi012aa1n06x5               g125(.a(new_n187), .b(new_n118), .c(new_n106), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n211), .o1(new_n222));
  oaoi13aa1n03x5               g127(.a(new_n222), .b(new_n207), .c(new_n221), .d(new_n201), .o1(new_n223));
  oaoi13aa1n02x5               g128(.a(new_n219), .b(new_n216), .c(new_n223), .d(new_n217), .o1(new_n224));
  norp02aa1n02x5               g129(.a(new_n224), .b(new_n220), .o1(\s[20] ));
  inv000aa1d42x5               g130(.a(\a[21] ), .o1(new_n226));
  nona22aa1d18x5               g131(.a(new_n207), .b(new_n217), .c(new_n219), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  norp02aa1n02x5               g133(.a(\b[19] ), .b(\a[20] ), .o1(new_n229));
  inv040aa1n02x5               g134(.a(new_n229), .o1(new_n230));
  and002aa1n02x5               g135(.a(\b[18] ), .b(\a[19] ), .o(new_n231));
  and002aa1n02x5               g136(.a(\b[19] ), .b(\a[20] ), .o(new_n232));
  nanp02aa1n09x5               g137(.a(new_n211), .b(new_n216), .o1(new_n233));
  nona22aa1n09x5               g138(.a(new_n233), .b(new_n232), .c(new_n231), .out0(new_n234));
  nand22aa1n04x5               g139(.a(new_n234), .b(new_n230), .o1(new_n235));
  oaoi13aa1n09x5               g140(.a(new_n235), .b(new_n228), .c(new_n221), .d(new_n201), .o1(new_n236));
  xorb03aa1n03x5               g141(.a(new_n236), .b(\b[20] ), .c(new_n226), .out0(\s[21] ));
  nanb02aa1n02x5               g142(.a(\b[20] ), .b(new_n226), .out0(new_n238));
  inv000aa1d42x5               g143(.a(new_n235), .o1(new_n239));
  xorc02aa1n12x5               g144(.a(\a[21] ), .b(\b[20] ), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  oaoi13aa1n06x5               g146(.a(new_n241), .b(new_n239), .c(new_n202), .d(new_n227), .o1(new_n242));
  xorc02aa1n12x5               g147(.a(\a[22] ), .b(\b[21] ), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  nano22aa1n02x4               g149(.a(new_n242), .b(new_n238), .c(new_n244), .out0(new_n245));
  oaoi13aa1n09x5               g150(.a(new_n244), .b(new_n238), .c(new_n236), .d(new_n241), .o1(new_n246));
  norp02aa1n02x5               g151(.a(new_n246), .b(new_n245), .o1(\s[22] ));
  inv000aa1d42x5               g152(.a(\a[23] ), .o1(new_n248));
  inv000aa1d42x5               g153(.a(\a[22] ), .o1(new_n249));
  xroi22aa1d06x4               g154(.a(new_n226), .b(\b[20] ), .c(new_n249), .d(\b[21] ), .out0(new_n250));
  nano32aa1n02x4               g155(.a(new_n219), .b(new_n250), .c(new_n207), .d(new_n212), .out0(new_n251));
  nanp02aa1n02x5               g156(.a(\b[21] ), .b(\a[22] ), .o1(new_n252));
  aoi112aa1n02x5               g157(.a(new_n232), .b(new_n231), .c(new_n211), .d(new_n216), .o1(new_n253));
  oai012aa1n02x5               g158(.a(new_n250), .b(new_n253), .c(new_n229), .o1(new_n254));
  oai022aa1d18x5               g159(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n255));
  aob012aa1n02x5               g160(.a(new_n254), .b(new_n255), .c(new_n252), .out0(new_n256));
  oaoi13aa1n06x5               g161(.a(new_n256), .b(new_n251), .c(new_n221), .d(new_n201), .o1(new_n257));
  xorb03aa1n02x5               g162(.a(new_n257), .b(\b[22] ), .c(new_n248), .out0(\s[23] ));
  inv000aa1d42x5               g163(.a(\b[22] ), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(new_n259), .b(new_n248), .o1(new_n260));
  aoai13aa1n06x5               g165(.a(new_n251), .b(new_n201), .c(new_n119), .d(new_n199), .o1(new_n261));
  xnrc02aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .out0(new_n262));
  aoib12aa1n02x5               g167(.a(new_n262), .b(new_n261), .c(new_n256), .out0(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[23] ), .b(\a[24] ), .out0(new_n264));
  nano22aa1n02x4               g169(.a(new_n263), .b(new_n260), .c(new_n264), .out0(new_n265));
  oaoi13aa1n02x5               g170(.a(new_n264), .b(new_n260), .c(new_n257), .d(new_n262), .o1(new_n266));
  norp02aa1n02x5               g171(.a(new_n266), .b(new_n265), .o1(\s[24] ));
  nona22aa1n03x5               g172(.a(new_n250), .b(new_n262), .c(new_n264), .out0(new_n268));
  nor042aa1n02x5               g173(.a(new_n227), .b(new_n268), .o1(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n201), .c(new_n119), .d(new_n199), .o1(new_n270));
  norp02aa1n02x5               g175(.a(\b[23] ), .b(\a[24] ), .o1(new_n271));
  oai112aa1n03x5               g176(.a(new_n255), .b(new_n252), .c(new_n259), .d(new_n248), .o1(new_n272));
  aoi022aa1n02x5               g177(.a(new_n272), .b(new_n260), .c(\a[24] ), .d(\b[23] ), .o1(new_n273));
  nor022aa1n04x5               g178(.a(new_n273), .b(new_n271), .o1(new_n274));
  aoai13aa1n12x5               g179(.a(new_n274), .b(new_n268), .c(new_n234), .d(new_n230), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  xnrc02aa1n12x5               g181(.a(\b[24] ), .b(\a[25] ), .out0(new_n277));
  xobna2aa1n03x5               g182(.a(new_n277), .b(new_n270), .c(new_n276), .out0(\s[25] ));
  nor042aa1n03x5               g183(.a(\b[24] ), .b(\a[25] ), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  tech160nm_fiaoi012aa1n03p5x5 g185(.a(new_n277), .b(new_n270), .c(new_n276), .o1(new_n281));
  xnrc02aa1n06x5               g186(.a(\b[25] ), .b(\a[26] ), .out0(new_n282));
  nano22aa1n02x4               g187(.a(new_n281), .b(new_n280), .c(new_n282), .out0(new_n283));
  oaoi13aa1n02x7               g188(.a(new_n275), .b(new_n269), .c(new_n221), .d(new_n201), .o1(new_n284));
  oaoi13aa1n03x5               g189(.a(new_n282), .b(new_n280), .c(new_n284), .d(new_n277), .o1(new_n285));
  norp02aa1n02x5               g190(.a(new_n285), .b(new_n283), .o1(\s[26] ));
  nor002aa1n02x5               g191(.a(new_n282), .b(new_n277), .o1(new_n287));
  inv000aa1n02x5               g192(.a(new_n287), .o1(new_n288));
  nor043aa1n06x5               g193(.a(new_n227), .b(new_n268), .c(new_n288), .o1(new_n289));
  aoai13aa1n06x5               g194(.a(new_n289), .b(new_n201), .c(new_n119), .d(new_n199), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[26] ), .b(\b[25] ), .c(new_n280), .carry(new_n291));
  aobi12aa1n12x5               g196(.a(new_n291), .b(new_n275), .c(new_n287), .out0(new_n292));
  xorc02aa1n12x5               g197(.a(\a[27] ), .b(\b[26] ), .out0(new_n293));
  xnbna2aa1n03x5               g198(.a(new_n293), .b(new_n292), .c(new_n290), .out0(\s[27] ));
  norp02aa1n02x5               g199(.a(\b[26] ), .b(\a[27] ), .o1(new_n295));
  inv040aa1n03x5               g200(.a(new_n295), .o1(new_n296));
  aobi12aa1n03x5               g201(.a(new_n293), .b(new_n292), .c(new_n290), .out0(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[27] ), .b(\a[28] ), .out0(new_n298));
  nano22aa1n03x5               g203(.a(new_n297), .b(new_n296), .c(new_n298), .out0(new_n299));
  nand22aa1n03x5               g204(.a(new_n243), .b(new_n240), .o1(new_n300));
  xorc02aa1n02x5               g205(.a(\a[23] ), .b(\b[22] ), .out0(new_n301));
  xorc02aa1n02x5               g206(.a(\a[24] ), .b(\b[23] ), .out0(new_n302));
  nano22aa1n03x7               g207(.a(new_n300), .b(new_n301), .c(new_n302), .out0(new_n303));
  tech160nm_fioai012aa1n05x5   g208(.a(new_n303), .b(new_n253), .c(new_n229), .o1(new_n304));
  aoai13aa1n04x5               g209(.a(new_n291), .b(new_n288), .c(new_n304), .d(new_n274), .o1(new_n305));
  aoai13aa1n02x5               g210(.a(new_n293), .b(new_n305), .c(new_n197), .d(new_n289), .o1(new_n306));
  aoi012aa1n02x5               g211(.a(new_n298), .b(new_n306), .c(new_n296), .o1(new_n307));
  norp02aa1n03x5               g212(.a(new_n307), .b(new_n299), .o1(\s[28] ));
  norb02aa1n02x5               g213(.a(new_n293), .b(new_n298), .out0(new_n309));
  aoai13aa1n02x5               g214(.a(new_n309), .b(new_n305), .c(new_n197), .d(new_n289), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .c(new_n296), .carry(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[28] ), .b(\a[29] ), .out0(new_n312));
  aoi012aa1n02x5               g217(.a(new_n312), .b(new_n310), .c(new_n311), .o1(new_n313));
  aobi12aa1n03x5               g218(.a(new_n309), .b(new_n292), .c(new_n290), .out0(new_n314));
  nano22aa1n03x5               g219(.a(new_n314), .b(new_n311), .c(new_n312), .out0(new_n315));
  norp02aa1n03x5               g220(.a(new_n313), .b(new_n315), .o1(\s[29] ));
  xnrb03aa1n02x5               g221(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g222(.a(new_n293), .b(new_n312), .c(new_n298), .out0(new_n318));
  aoai13aa1n02x5               g223(.a(new_n318), .b(new_n305), .c(new_n197), .d(new_n289), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .c(new_n311), .carry(new_n320));
  xnrc02aa1n02x5               g225(.a(\b[29] ), .b(\a[30] ), .out0(new_n321));
  aoi012aa1n02x5               g226(.a(new_n321), .b(new_n319), .c(new_n320), .o1(new_n322));
  aobi12aa1n03x5               g227(.a(new_n318), .b(new_n292), .c(new_n290), .out0(new_n323));
  nano22aa1n03x5               g228(.a(new_n323), .b(new_n320), .c(new_n321), .out0(new_n324));
  norp02aa1n03x5               g229(.a(new_n322), .b(new_n324), .o1(\s[30] ));
  xnrc02aa1n02x5               g230(.a(\b[30] ), .b(\a[31] ), .out0(new_n326));
  norb02aa1n02x5               g231(.a(new_n318), .b(new_n321), .out0(new_n327));
  aoai13aa1n02x5               g232(.a(new_n327), .b(new_n305), .c(new_n197), .d(new_n289), .o1(new_n328));
  oao003aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .c(new_n320), .carry(new_n329));
  aoi012aa1n02x5               g234(.a(new_n326), .b(new_n328), .c(new_n329), .o1(new_n330));
  aobi12aa1n03x5               g235(.a(new_n327), .b(new_n292), .c(new_n290), .out0(new_n331));
  nano22aa1n03x5               g236(.a(new_n331), .b(new_n326), .c(new_n329), .out0(new_n332));
  norp02aa1n03x5               g237(.a(new_n330), .b(new_n332), .o1(\s[31] ));
  xnrb03aa1n02x5               g238(.a(new_n153), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  inv000aa1d42x5               g239(.a(new_n111), .o1(new_n335));
  aoi122aa1n02x5               g240(.a(new_n113), .b(new_n335), .c(new_n112), .d(new_n110), .e(new_n114), .o1(new_n336));
  aoi012aa1n02x5               g241(.a(new_n336), .b(new_n335), .c(new_n155), .o1(\s[4] ));
  xorb03aa1n02x5               g242(.a(new_n155), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1n06x5               g243(.a(new_n155), .o1(new_n339));
  oaoi03aa1n09x5               g244(.a(\a[5] ), .b(\b[4] ), .c(new_n339), .o1(new_n340));
  xorb03aa1n02x5               g245(.a(new_n340), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  orn002aa1n02x5               g246(.a(\a[6] ), .b(\b[5] ), .o(new_n342));
  inv000aa1d42x5               g247(.a(new_n342), .o1(new_n343));
  nanp02aa1n02x5               g248(.a(\b[5] ), .b(\a[6] ), .o1(new_n344));
  aoi112aa1n02x5               g249(.a(new_n343), .b(new_n102), .c(new_n340), .d(new_n344), .o1(new_n345));
  aoai13aa1n02x5               g250(.a(new_n102), .b(new_n343), .c(new_n340), .d(new_n344), .o1(new_n346));
  norb02aa1n02x5               g251(.a(new_n346), .b(new_n345), .out0(\s[7] ));
  xorc02aa1n02x5               g252(.a(\a[8] ), .b(\b[7] ), .out0(new_n348));
  xnbna2aa1n03x5               g253(.a(new_n348), .b(new_n346), .c(new_n104), .out0(\s[8] ));
  xorb03aa1n02x5               g254(.a(new_n119), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule

