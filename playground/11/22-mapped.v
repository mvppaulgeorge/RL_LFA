// Benchmark "adder" written by ABC on Wed Jul 17 17:46:09 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n255, new_n256, new_n257, new_n258, new_n259, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n315, new_n318, new_n320,
    new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  and002aa1n02x5               g002(.a(\b[3] ), .b(\a[4] ), .o(new_n98));
  xnrc02aa1n12x5               g003(.a(\b[2] ), .b(\a[3] ), .out0(new_n99));
  nand42aa1n06x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nor042aa1d18x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nand02aa1d28x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  oai012aa1n18x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  oa0022aa1n06x5               g008(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n104));
  oaih12aa1n02x5               g009(.a(new_n104), .b(new_n99), .c(new_n103), .o1(new_n105));
  nanb02aa1n02x5               g010(.a(new_n98), .b(new_n105), .out0(new_n106));
  nor002aa1n16x5               g011(.a(\b[7] ), .b(\a[8] ), .o1(new_n107));
  nand42aa1n08x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nor022aa1n16x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nand42aa1n08x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nano23aa1d15x5               g015(.a(new_n107), .b(new_n109), .c(new_n110), .d(new_n108), .out0(new_n111));
  xorc02aa1n02x5               g016(.a(\a[6] ), .b(\b[5] ), .out0(new_n112));
  xorc02aa1n02x5               g017(.a(\a[5] ), .b(\b[4] ), .out0(new_n113));
  nanp03aa1n02x5               g018(.a(new_n111), .b(new_n112), .c(new_n113), .o1(new_n114));
  and002aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .o(new_n115));
  inv000aa1n02x5               g020(.a(new_n115), .o1(new_n116));
  aoi112aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n117));
  inv000aa1d42x5               g022(.a(\a[5] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\a[6] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[4] ), .o1(new_n120));
  aboi22aa1d24x5               g025(.a(\b[5] ), .b(new_n119), .c(new_n118), .d(new_n120), .out0(new_n121));
  inv000aa1d42x5               g026(.a(new_n121), .o1(new_n122));
  aoi113aa1n03x7               g027(.a(new_n107), .b(new_n117), .c(new_n111), .d(new_n116), .e(new_n122), .o1(new_n123));
  oai012aa1n12x5               g028(.a(new_n123), .b(new_n106), .c(new_n114), .o1(new_n124));
  tech160nm_fixnrc02aa1n05x5   g029(.a(\b[8] ), .b(\a[9] ), .out0(new_n125));
  aoib12aa1n02x5               g030(.a(new_n97), .b(new_n124), .c(new_n125), .out0(new_n126));
  xnrb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  tech160nm_fixnrc02aa1n05x5   g032(.a(\b[9] ), .b(\a[10] ), .out0(new_n128));
  nor042aa1n02x5               g033(.a(new_n125), .b(new_n128), .o1(new_n129));
  norp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  aoi112aa1n03x5               g035(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n131));
  nor042aa1n03x5               g036(.a(new_n131), .b(new_n130), .o1(new_n132));
  aob012aa1n02x5               g037(.a(new_n132), .b(new_n124), .c(new_n129), .out0(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n06x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand22aa1n09x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nor042aa1n06x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n09x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  aoi112aa1n02x5               g044(.a(new_n139), .b(new_n135), .c(new_n133), .d(new_n136), .o1(new_n140));
  aoai13aa1n02x7               g045(.a(new_n139), .b(new_n135), .c(new_n133), .d(new_n136), .o1(new_n141));
  norb02aa1n03x4               g046(.a(new_n141), .b(new_n140), .out0(\s[12] ));
  oaoi13aa1n12x5               g047(.a(new_n98), .b(new_n104), .c(new_n99), .d(new_n103), .o1(new_n143));
  nona23aa1n02x4               g048(.a(new_n110), .b(new_n108), .c(new_n107), .d(new_n109), .out0(new_n144));
  nano22aa1n03x7               g049(.a(new_n144), .b(new_n112), .c(new_n113), .out0(new_n145));
  nona22aa1n03x5               g050(.a(new_n111), .b(new_n115), .c(new_n121), .out0(new_n146));
  nona22aa1n03x5               g051(.a(new_n146), .b(new_n117), .c(new_n107), .out0(new_n147));
  nona23aa1n09x5               g052(.a(new_n138), .b(new_n136), .c(new_n135), .d(new_n137), .out0(new_n148));
  norp03aa1n02x5               g053(.a(new_n148), .b(new_n125), .c(new_n128), .o1(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n147), .c(new_n143), .d(new_n145), .o1(new_n150));
  tech160nm_fioai012aa1n03p5x5 g055(.a(new_n138), .b(new_n137), .c(new_n135), .o1(new_n151));
  oai012aa1n18x5               g056(.a(new_n151), .b(new_n148), .c(new_n132), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n150), .b(new_n153), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand42aa1n20x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n156), .b(new_n154), .c(new_n157), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n06x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand02aa1d16x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nona23aa1n02x4               g066(.a(new_n161), .b(new_n157), .c(new_n156), .d(new_n160), .out0(new_n162));
  tech160nm_fiaoi012aa1n04x5   g067(.a(new_n160), .b(new_n156), .c(new_n161), .o1(new_n163));
  aoai13aa1n06x5               g068(.a(new_n163), .b(new_n162), .c(new_n150), .d(new_n153), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  xorc02aa1n12x5               g071(.a(\a[15] ), .b(\b[14] ), .out0(new_n167));
  xorc02aa1n12x5               g072(.a(\a[16] ), .b(\b[15] ), .out0(new_n168));
  aoi112aa1n02x5               g073(.a(new_n168), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n169));
  aoai13aa1n03x5               g074(.a(new_n168), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(\s[16] ));
  nano23aa1n09x5               g076(.a(new_n135), .b(new_n137), .c(new_n138), .d(new_n136), .out0(new_n172));
  nano23aa1n03x7               g077(.a(new_n156), .b(new_n160), .c(new_n161), .d(new_n157), .out0(new_n173));
  nand23aa1n02x5               g078(.a(new_n173), .b(new_n167), .c(new_n168), .o1(new_n174));
  nano22aa1n03x7               g079(.a(new_n174), .b(new_n129), .c(new_n172), .out0(new_n175));
  aoai13aa1n06x5               g080(.a(new_n175), .b(new_n147), .c(new_n143), .d(new_n145), .o1(new_n176));
  nor002aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  inv000aa1n02x5               g082(.a(new_n177), .o1(new_n178));
  nanb03aa1n03x5               g083(.a(new_n163), .b(new_n168), .c(new_n167), .out0(new_n179));
  aoi112aa1n03x5               g084(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n180));
  inv000aa1n02x5               g085(.a(new_n180), .o1(new_n181));
  oai012aa1n02x5               g086(.a(new_n172), .b(new_n131), .c(new_n130), .o1(new_n182));
  aoi012aa1n03x5               g087(.a(new_n174), .b(new_n182), .c(new_n151), .o1(new_n183));
  nano32aa1n03x7               g088(.a(new_n183), .b(new_n181), .c(new_n179), .d(new_n178), .out0(new_n184));
  nand02aa1d08x5               g089(.a(new_n176), .b(new_n184), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g091(.a(\a[18] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\a[17] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[16] ), .o1(new_n189));
  oaoi03aa1n03x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  xroi22aa1d06x4               g096(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n192));
  nand02aa1d04x5               g097(.a(\b[17] ), .b(\a[18] ), .o1(new_n193));
  nona22aa1n02x4               g098(.a(new_n193), .b(\b[16] ), .c(\a[17] ), .out0(new_n194));
  oaib12aa1n06x5               g099(.a(new_n194), .b(\b[17] ), .c(new_n187), .out0(new_n195));
  norp02aa1n12x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nand42aa1n06x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n195), .c(new_n185), .d(new_n192), .o1(new_n199));
  aoi112aa1n02x5               g104(.a(new_n198), .b(new_n195), .c(new_n185), .d(new_n192), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n199), .b(new_n200), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n08x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nand42aa1n04x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  nona22aa1n06x5               g110(.a(new_n199), .b(new_n205), .c(new_n196), .out0(new_n206));
  orn002aa1n24x5               g111(.a(\a[19] ), .b(\b[18] ), .o(new_n207));
  aobi12aa1n06x5               g112(.a(new_n205), .b(new_n199), .c(new_n207), .out0(new_n208));
  norb02aa1n03x4               g113(.a(new_n206), .b(new_n208), .out0(\s[20] ));
  nano23aa1n09x5               g114(.a(new_n196), .b(new_n203), .c(new_n204), .d(new_n197), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n192), .b(new_n210), .o1(new_n211));
  norp02aa1n02x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  aoi013aa1n06x4               g117(.a(new_n212), .b(new_n193), .c(new_n188), .d(new_n189), .o1(new_n213));
  nona23aa1n09x5               g118(.a(new_n204), .b(new_n197), .c(new_n196), .d(new_n203), .out0(new_n214));
  oaoi03aa1n09x5               g119(.a(\a[20] ), .b(\b[19] ), .c(new_n207), .o1(new_n215));
  inv040aa1n02x5               g120(.a(new_n215), .o1(new_n216));
  oai012aa1d24x5               g121(.a(new_n216), .b(new_n214), .c(new_n213), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n03x5               g123(.a(new_n218), .b(new_n211), .c(new_n176), .d(new_n184), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xorc02aa1n02x5               g127(.a(\a[22] ), .b(\b[21] ), .out0(new_n223));
  aoi112aa1n02x7               g128(.a(new_n221), .b(new_n223), .c(new_n219), .d(new_n222), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n223), .b(new_n221), .c(new_n219), .d(new_n222), .o1(new_n225));
  norb02aa1n03x4               g130(.a(new_n225), .b(new_n224), .out0(\s[22] ));
  inv000aa1d42x5               g131(.a(\a[21] ), .o1(new_n227));
  inv040aa1d32x5               g132(.a(\a[22] ), .o1(new_n228));
  xroi22aa1d06x4               g133(.a(new_n227), .b(\b[20] ), .c(new_n228), .d(\b[21] ), .out0(new_n229));
  nand23aa1n03x5               g134(.a(new_n229), .b(new_n192), .c(new_n210), .o1(new_n230));
  inv000aa1d42x5               g135(.a(\b[21] ), .o1(new_n231));
  oaoi03aa1n09x5               g136(.a(new_n228), .b(new_n231), .c(new_n221), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoi012aa1n02x5               g138(.a(new_n233), .b(new_n217), .c(new_n229), .o1(new_n234));
  aoai13aa1n06x5               g139(.a(new_n234), .b(new_n230), .c(new_n176), .d(new_n184), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  tech160nm_fixorc02aa1n04x5   g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  nor002aa1n02x5               g143(.a(\b[23] ), .b(\a[24] ), .o1(new_n239));
  nand42aa1n06x5               g144(.a(\b[23] ), .b(\a[24] ), .o1(new_n240));
  norb02aa1n06x4               g145(.a(new_n240), .b(new_n239), .out0(new_n241));
  aoi112aa1n03x5               g146(.a(new_n237), .b(new_n241), .c(new_n235), .d(new_n238), .o1(new_n242));
  aoai13aa1n03x5               g147(.a(new_n241), .b(new_n237), .c(new_n235), .d(new_n238), .o1(new_n243));
  norb02aa1n03x4               g148(.a(new_n243), .b(new_n242), .out0(\s[24] ));
  and002aa1n06x5               g149(.a(new_n238), .b(new_n241), .o(new_n245));
  inv040aa1n02x5               g150(.a(new_n245), .o1(new_n246));
  nano32aa1n02x4               g151(.a(new_n246), .b(new_n229), .c(new_n192), .d(new_n210), .out0(new_n247));
  aoai13aa1n03x5               g152(.a(new_n229), .b(new_n215), .c(new_n210), .d(new_n195), .o1(new_n248));
  oai012aa1n02x5               g153(.a(new_n240), .b(new_n239), .c(new_n237), .o1(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n246), .c(new_n248), .d(new_n232), .o1(new_n250));
  xorc02aa1n02x5               g155(.a(\a[25] ), .b(\b[24] ), .out0(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n250), .c(new_n185), .d(new_n247), .o1(new_n252));
  aoi112aa1n02x5               g157(.a(new_n251), .b(new_n250), .c(new_n185), .d(new_n247), .o1(new_n253));
  norb02aa1n02x5               g158(.a(new_n252), .b(new_n253), .out0(\s[25] ));
  nor042aa1n03x5               g159(.a(\b[24] ), .b(\a[25] ), .o1(new_n255));
  xorc02aa1n02x5               g160(.a(\a[26] ), .b(\b[25] ), .out0(new_n256));
  nona22aa1n06x5               g161(.a(new_n252), .b(new_n256), .c(new_n255), .out0(new_n257));
  inv000aa1d42x5               g162(.a(new_n255), .o1(new_n258));
  aobi12aa1n06x5               g163(.a(new_n256), .b(new_n252), .c(new_n258), .out0(new_n259));
  norb02aa1n03x4               g164(.a(new_n257), .b(new_n259), .out0(\s[26] ));
  xnrc02aa1n02x5               g165(.a(\b[14] ), .b(\a[15] ), .out0(new_n261));
  xnrc02aa1n02x5               g166(.a(\b[15] ), .b(\a[16] ), .out0(new_n262));
  norp03aa1n02x5               g167(.a(new_n262), .b(new_n261), .c(new_n163), .o1(new_n263));
  nona32aa1n09x5               g168(.a(new_n152), .b(new_n262), .c(new_n162), .d(new_n261), .out0(new_n264));
  nona32aa1n03x5               g169(.a(new_n264), .b(new_n180), .c(new_n263), .d(new_n177), .out0(new_n265));
  inv000aa1d42x5               g170(.a(\a[25] ), .o1(new_n266));
  inv020aa1n04x5               g171(.a(\a[26] ), .o1(new_n267));
  xroi22aa1d06x4               g172(.a(new_n266), .b(\b[24] ), .c(new_n267), .d(\b[25] ), .out0(new_n268));
  nano22aa1d15x5               g173(.a(new_n230), .b(new_n245), .c(new_n268), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n265), .c(new_n124), .d(new_n175), .o1(new_n270));
  oao003aa1n02x5               g175(.a(\a[26] ), .b(\b[25] ), .c(new_n258), .carry(new_n271));
  aobi12aa1n06x5               g176(.a(new_n271), .b(new_n250), .c(new_n268), .out0(new_n272));
  nor042aa1n03x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  nanp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  norb02aa1n02x5               g179(.a(new_n274), .b(new_n273), .out0(new_n275));
  xnbna2aa1n03x5               g180(.a(new_n275), .b(new_n272), .c(new_n270), .out0(\s[27] ));
  xorc02aa1n02x5               g181(.a(\a[28] ), .b(\b[27] ), .out0(new_n277));
  aoai13aa1n03x5               g182(.a(new_n245), .b(new_n233), .c(new_n217), .d(new_n229), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n268), .o1(new_n279));
  aoai13aa1n06x5               g184(.a(new_n271), .b(new_n279), .c(new_n278), .d(new_n249), .o1(new_n280));
  aoi112aa1n02x5               g185(.a(new_n280), .b(new_n273), .c(new_n185), .d(new_n269), .o1(new_n281));
  nano22aa1n03x5               g186(.a(new_n281), .b(new_n274), .c(new_n277), .out0(new_n282));
  inv000aa1n03x5               g187(.a(new_n273), .o1(new_n283));
  nanp03aa1n03x5               g188(.a(new_n272), .b(new_n270), .c(new_n283), .o1(new_n284));
  aoi012aa1n03x5               g189(.a(new_n277), .b(new_n284), .c(new_n274), .o1(new_n285));
  nor002aa1n02x5               g190(.a(new_n285), .b(new_n282), .o1(\s[28] ));
  inv020aa1n03x5               g191(.a(new_n269), .o1(new_n287));
  aoi012aa1n06x5               g192(.a(new_n287), .b(new_n176), .c(new_n184), .o1(new_n288));
  and002aa1n02x5               g193(.a(new_n277), .b(new_n275), .o(new_n289));
  oaih12aa1n02x5               g194(.a(new_n289), .b(new_n280), .c(new_n288), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[28] ), .b(\a[29] ), .out0(new_n292));
  tech160nm_fiaoi012aa1n02p5x5 g197(.a(new_n292), .b(new_n290), .c(new_n291), .o1(new_n293));
  aobi12aa1n02x7               g198(.a(new_n289), .b(new_n272), .c(new_n270), .out0(new_n294));
  nano22aa1n03x5               g199(.a(new_n294), .b(new_n291), .c(new_n292), .out0(new_n295));
  norp02aa1n03x5               g200(.a(new_n293), .b(new_n295), .o1(\s[29] ));
  xorb03aa1n02x5               g201(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g202(.a(new_n292), .b(new_n277), .c(new_n275), .out0(new_n298));
  oaih12aa1n02x5               g203(.a(new_n298), .b(new_n280), .c(new_n288), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[29] ), .b(\a[30] ), .out0(new_n301));
  tech160nm_fiaoi012aa1n02p5x5 g206(.a(new_n301), .b(new_n299), .c(new_n300), .o1(new_n302));
  aobi12aa1n02x7               g207(.a(new_n298), .b(new_n272), .c(new_n270), .out0(new_n303));
  nano22aa1n03x5               g208(.a(new_n303), .b(new_n300), .c(new_n301), .out0(new_n304));
  norp02aa1n03x5               g209(.a(new_n302), .b(new_n304), .o1(\s[30] ));
  nano23aa1n02x4               g210(.a(new_n301), .b(new_n292), .c(new_n277), .d(new_n275), .out0(new_n306));
  aobi12aa1n02x7               g211(.a(new_n306), .b(new_n272), .c(new_n270), .out0(new_n307));
  oao003aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .c(new_n300), .carry(new_n308));
  xnrc02aa1n02x5               g213(.a(\b[30] ), .b(\a[31] ), .out0(new_n309));
  nano22aa1n03x5               g214(.a(new_n307), .b(new_n308), .c(new_n309), .out0(new_n310));
  oaih12aa1n02x5               g215(.a(new_n306), .b(new_n280), .c(new_n288), .o1(new_n311));
  tech160nm_fiaoi012aa1n02p5x5 g216(.a(new_n309), .b(new_n311), .c(new_n308), .o1(new_n312));
  norp02aa1n03x5               g217(.a(new_n312), .b(new_n310), .o1(\s[31] ));
  xnrb03aa1n02x5               g218(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g219(.a(\a[3] ), .b(\b[2] ), .c(new_n103), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g221(.a(new_n143), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g222(.a(new_n118), .b(new_n120), .c(new_n143), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(new_n119), .out0(\s[6] ));
  oaoi03aa1n02x5               g224(.a(\a[6] ), .b(\b[5] ), .c(new_n318), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g226(.a(new_n109), .b(new_n320), .c(new_n110), .o1(new_n322));
  xnrb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g228(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


