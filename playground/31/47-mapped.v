// Benchmark "adder" written by ABC on Thu Jul 18 04:17:05 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n316, new_n319,
    new_n321, new_n323;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv020aa1n02x5               g002(.a(new_n97), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1d04x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  tech160nm_fioai012aa1n05x5   g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  nor022aa1n08x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  tech160nm_finand02aa1n03p5x5 g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n16x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  tech160nm_fioai012aa1n03p5x5 g012(.a(new_n104), .b(new_n105), .c(new_n103), .o1(new_n108));
  oai012aa1n09x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nand42aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor022aa1n16x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor022aa1n06x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand22aa1n03x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x5               g018(.a(new_n110), .b(new_n113), .c(new_n112), .d(new_n111), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nor043aa1n03x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  aoi112aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n118));
  nano23aa1n03x7               g023(.a(new_n112), .b(new_n111), .c(new_n113), .d(new_n110), .out0(new_n119));
  and002aa1n24x5               g024(.a(\b[5] ), .b(\a[6] ), .o(new_n120));
  nor042aa1n09x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  oab012aa1d24x5               g026(.a(new_n121), .b(\a[6] ), .c(\b[5] ), .out0(new_n122));
  nona22aa1n06x5               g027(.a(new_n119), .b(new_n120), .c(new_n122), .out0(new_n123));
  nona22aa1n06x5               g028(.a(new_n123), .b(new_n118), .c(new_n111), .out0(new_n124));
  xorc02aa1n12x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n126));
  xorc02aa1n12x5               g031(.a(\a[10] ), .b(\b[9] ), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g033(.a(new_n127), .o1(new_n129));
  oaoi03aa1n02x5               g034(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n130));
  inv000aa1n02x5               g035(.a(new_n130), .o1(new_n131));
  aoai13aa1n04x5               g036(.a(new_n131), .b(new_n129), .c(new_n126), .d(new_n98), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n06x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand02aa1d16x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nor042aa1n09x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nand02aa1d08x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  norb02aa1n06x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  aoi112aa1n02x5               g043(.a(new_n138), .b(new_n134), .c(new_n132), .d(new_n135), .o1(new_n139));
  aoai13aa1n02x5               g044(.a(new_n138), .b(new_n134), .c(new_n132), .d(new_n135), .o1(new_n140));
  norb02aa1n02x7               g045(.a(new_n140), .b(new_n139), .out0(\s[12] ));
  nano23aa1n03x7               g046(.a(new_n134), .b(new_n136), .c(new_n137), .d(new_n135), .out0(new_n142));
  nand23aa1n02x5               g047(.a(new_n142), .b(new_n125), .c(new_n127), .o1(new_n143));
  inv000aa1n02x5               g048(.a(new_n143), .o1(new_n144));
  aoai13aa1n04x5               g049(.a(new_n144), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n145));
  aoi112aa1n09x5               g050(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n146));
  nor042aa1n02x5               g051(.a(\b[9] ), .b(\a[10] ), .o1(new_n147));
  norb02aa1n06x5               g052(.a(new_n135), .b(new_n134), .out0(new_n148));
  aoi112aa1n09x5               g053(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n149));
  oai112aa1n06x5               g054(.a(new_n138), .b(new_n148), .c(new_n149), .d(new_n147), .o1(new_n150));
  nona22aa1d24x5               g055(.a(new_n150), .b(new_n146), .c(new_n136), .out0(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  nor002aa1n08x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  tech160nm_finand02aa1n03p5x5 g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanb02aa1n02x5               g059(.a(new_n153), .b(new_n154), .out0(new_n155));
  xobna2aa1n03x5               g060(.a(new_n155), .b(new_n145), .c(new_n152), .out0(\s[13] ));
  nand42aa1n02x5               g061(.a(new_n109), .b(new_n117), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n120), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n122), .o1(new_n159));
  aoi113aa1n02x5               g064(.a(new_n111), .b(new_n118), .c(new_n119), .d(new_n158), .e(new_n159), .o1(new_n160));
  aoai13aa1n02x5               g065(.a(new_n152), .b(new_n143), .c(new_n160), .d(new_n157), .o1(new_n161));
  aoi012aa1n02x5               g066(.a(new_n153), .b(new_n161), .c(new_n154), .o1(new_n162));
  xnrb03aa1n02x5               g067(.a(new_n162), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n04x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand22aa1n03x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nona23aa1n12x5               g070(.a(new_n165), .b(new_n154), .c(new_n153), .d(new_n164), .out0(new_n166));
  aoi012aa1n02x7               g071(.a(new_n164), .b(new_n153), .c(new_n165), .o1(new_n167));
  aoai13aa1n04x5               g072(.a(new_n167), .b(new_n166), .c(new_n145), .d(new_n152), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n04x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nand42aa1n04x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nor002aa1d32x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nand42aa1n06x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanb02aa1n18x5               g078(.a(new_n172), .b(new_n173), .out0(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  aoi112aa1n02x5               g080(.a(new_n175), .b(new_n170), .c(new_n168), .d(new_n171), .o1(new_n176));
  aoai13aa1n04x5               g081(.a(new_n175), .b(new_n170), .c(new_n168), .d(new_n171), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(\s[16] ));
  nand02aa1d04x5               g083(.a(new_n127), .b(new_n125), .o1(new_n179));
  nano23aa1n02x5               g084(.a(new_n170), .b(new_n172), .c(new_n173), .d(new_n171), .out0(new_n180));
  nano23aa1n06x5               g085(.a(new_n166), .b(new_n179), .c(new_n180), .d(new_n142), .out0(new_n181));
  aoai13aa1n06x5               g086(.a(new_n181), .b(new_n124), .c(new_n109), .d(new_n117), .o1(new_n182));
  nanb02aa1n02x5               g087(.a(new_n170), .b(new_n171), .out0(new_n183));
  nor003aa1n03x5               g088(.a(new_n166), .b(new_n174), .c(new_n183), .o1(new_n184));
  aoi112aa1n02x5               g089(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n172), .o1(new_n186));
  oai013aa1n03x5               g091(.a(new_n186), .b(new_n167), .c(new_n183), .d(new_n174), .o1(new_n187));
  aoi112aa1n09x5               g092(.a(new_n187), .b(new_n185), .c(new_n151), .d(new_n184), .o1(new_n188));
  nand02aa1d08x5               g093(.a(new_n182), .b(new_n188), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g095(.a(\a[18] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(\a[17] ), .o1(new_n192));
  inv000aa1d42x5               g097(.a(\b[16] ), .o1(new_n193));
  oaoi03aa1n03x5               g098(.a(new_n192), .b(new_n193), .c(new_n189), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[17] ), .c(new_n191), .out0(\s[18] ));
  xroi22aa1d04x5               g100(.a(new_n192), .b(\b[16] ), .c(new_n191), .d(\b[17] ), .out0(new_n196));
  nand02aa1d04x5               g101(.a(\b[17] ), .b(\a[18] ), .o1(new_n197));
  nona22aa1n02x4               g102(.a(new_n197), .b(\b[16] ), .c(\a[17] ), .out0(new_n198));
  oaib12aa1n09x5               g103(.a(new_n198), .b(\b[17] ), .c(new_n191), .out0(new_n199));
  nor022aa1n08x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nand42aa1d28x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  aoai13aa1n06x5               g107(.a(new_n202), .b(new_n199), .c(new_n189), .d(new_n196), .o1(new_n203));
  aoi112aa1n02x5               g108(.a(new_n202), .b(new_n199), .c(new_n189), .d(new_n196), .o1(new_n204));
  norb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n04x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nand42aa1n10x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  nona22aa1n02x5               g114(.a(new_n203), .b(new_n209), .c(new_n200), .out0(new_n210));
  orn002aa1n24x5               g115(.a(\a[19] ), .b(\b[18] ), .o(new_n211));
  aobi12aa1n02x7               g116(.a(new_n209), .b(new_n203), .c(new_n211), .out0(new_n212));
  norb02aa1n03x4               g117(.a(new_n210), .b(new_n212), .out0(\s[20] ));
  nano23aa1n06x5               g118(.a(new_n200), .b(new_n207), .c(new_n208), .d(new_n201), .out0(new_n214));
  nanp02aa1n02x5               g119(.a(new_n196), .b(new_n214), .o1(new_n215));
  norp02aa1n02x5               g120(.a(\b[17] ), .b(\a[18] ), .o1(new_n216));
  aoi013aa1n06x5               g121(.a(new_n216), .b(new_n197), .c(new_n192), .d(new_n193), .o1(new_n217));
  nona23aa1d18x5               g122(.a(new_n208), .b(new_n201), .c(new_n200), .d(new_n207), .out0(new_n218));
  oaoi03aa1n12x5               g123(.a(\a[20] ), .b(\b[19] ), .c(new_n211), .o1(new_n219));
  inv000aa1n03x5               g124(.a(new_n219), .o1(new_n220));
  oai012aa1n18x5               g125(.a(new_n220), .b(new_n218), .c(new_n217), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  aoai13aa1n06x5               g127(.a(new_n222), .b(new_n215), .c(new_n182), .d(new_n188), .o1(new_n223));
  xorb03aa1n02x5               g128(.a(new_n223), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  xorc02aa1n02x5               g130(.a(\a[21] ), .b(\b[20] ), .out0(new_n226));
  xorc02aa1n02x5               g131(.a(\a[22] ), .b(\b[21] ), .out0(new_n227));
  aoi112aa1n03x5               g132(.a(new_n225), .b(new_n227), .c(new_n223), .d(new_n226), .o1(new_n228));
  aoai13aa1n03x5               g133(.a(new_n227), .b(new_n225), .c(new_n223), .d(new_n226), .o1(new_n229));
  norb02aa1n03x4               g134(.a(new_n229), .b(new_n228), .out0(\s[22] ));
  inv000aa1d42x5               g135(.a(\a[21] ), .o1(new_n231));
  inv000aa1d42x5               g136(.a(\a[22] ), .o1(new_n232));
  xroi22aa1d06x4               g137(.a(new_n231), .b(\b[20] ), .c(new_n232), .d(\b[21] ), .out0(new_n233));
  nand43aa1n02x5               g138(.a(new_n233), .b(new_n196), .c(new_n214), .o1(new_n234));
  inv000aa1d42x5               g139(.a(\b[21] ), .o1(new_n235));
  oao003aa1n12x5               g140(.a(new_n232), .b(new_n235), .c(new_n225), .carry(new_n236));
  aoi012aa1n02x5               g141(.a(new_n236), .b(new_n221), .c(new_n233), .o1(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n234), .c(new_n182), .d(new_n188), .o1(new_n238));
  xorb03aa1n02x5               g143(.a(new_n238), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  xorc02aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  norp02aa1n02x5               g146(.a(\b[23] ), .b(\a[24] ), .o1(new_n242));
  nand42aa1n03x5               g147(.a(\b[23] ), .b(\a[24] ), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n242), .out0(new_n244));
  aoi112aa1n02x5               g149(.a(new_n240), .b(new_n244), .c(new_n238), .d(new_n241), .o1(new_n245));
  aoai13aa1n03x5               g150(.a(new_n244), .b(new_n240), .c(new_n238), .d(new_n241), .o1(new_n246));
  norb02aa1n02x5               g151(.a(new_n246), .b(new_n245), .out0(\s[24] ));
  and002aa1n06x5               g152(.a(new_n241), .b(new_n244), .o(new_n248));
  inv000aa1n02x5               g153(.a(new_n248), .o1(new_n249));
  nano32aa1n02x4               g154(.a(new_n249), .b(new_n233), .c(new_n196), .d(new_n214), .out0(new_n250));
  aoai13aa1n03x5               g155(.a(new_n233), .b(new_n219), .c(new_n214), .d(new_n199), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n236), .o1(new_n252));
  oai012aa1n02x5               g157(.a(new_n243), .b(new_n242), .c(new_n240), .o1(new_n253));
  aoai13aa1n06x5               g158(.a(new_n253), .b(new_n249), .c(new_n251), .d(new_n252), .o1(new_n254));
  xorc02aa1n02x5               g159(.a(\a[25] ), .b(\b[24] ), .out0(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n254), .c(new_n189), .d(new_n250), .o1(new_n256));
  aoi112aa1n02x5               g161(.a(new_n255), .b(new_n254), .c(new_n189), .d(new_n250), .o1(new_n257));
  norb02aa1n02x5               g162(.a(new_n256), .b(new_n257), .out0(\s[25] ));
  nor042aa1n03x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  xorc02aa1n02x5               g164(.a(\a[26] ), .b(\b[25] ), .out0(new_n260));
  nona22aa1n02x5               g165(.a(new_n256), .b(new_n260), .c(new_n259), .out0(new_n261));
  inv000aa1d42x5               g166(.a(new_n259), .o1(new_n262));
  aobi12aa1n02x7               g167(.a(new_n260), .b(new_n256), .c(new_n262), .out0(new_n263));
  norb02aa1n03x4               g168(.a(new_n261), .b(new_n263), .out0(\s[26] ));
  nand22aa1n03x5               g169(.a(new_n160), .b(new_n157), .o1(new_n265));
  nanp02aa1n02x5               g170(.a(new_n151), .b(new_n184), .o1(new_n266));
  nona22aa1n03x5               g171(.a(new_n266), .b(new_n187), .c(new_n185), .out0(new_n267));
  inv000aa1d42x5               g172(.a(\a[25] ), .o1(new_n268));
  inv020aa1n04x5               g173(.a(\a[26] ), .o1(new_n269));
  xroi22aa1d06x4               g174(.a(new_n268), .b(\b[24] ), .c(new_n269), .d(\b[25] ), .out0(new_n270));
  nano22aa1n03x7               g175(.a(new_n234), .b(new_n248), .c(new_n270), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n267), .c(new_n265), .d(new_n181), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[26] ), .b(\b[25] ), .c(new_n262), .carry(new_n273));
  aobi12aa1n06x5               g178(.a(new_n273), .b(new_n254), .c(new_n270), .out0(new_n274));
  xorc02aa1n02x5               g179(.a(\a[27] ), .b(\b[26] ), .out0(new_n275));
  xnbna2aa1n03x5               g180(.a(new_n275), .b(new_n274), .c(new_n272), .out0(\s[27] ));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  inv040aa1n03x5               g182(.a(new_n277), .o1(new_n278));
  aobi12aa1n02x7               g183(.a(new_n275), .b(new_n274), .c(new_n272), .out0(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .out0(new_n280));
  nano22aa1n03x5               g185(.a(new_n279), .b(new_n278), .c(new_n280), .out0(new_n281));
  inv020aa1n03x5               g186(.a(new_n271), .o1(new_n282));
  aoi012aa1n06x5               g187(.a(new_n282), .b(new_n182), .c(new_n188), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n248), .b(new_n236), .c(new_n221), .d(new_n233), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n270), .o1(new_n285));
  aoai13aa1n06x5               g190(.a(new_n273), .b(new_n285), .c(new_n284), .d(new_n253), .o1(new_n286));
  oaih12aa1n02x5               g191(.a(new_n275), .b(new_n286), .c(new_n283), .o1(new_n287));
  tech160nm_fiaoi012aa1n02p5x5 g192(.a(new_n280), .b(new_n287), .c(new_n278), .o1(new_n288));
  norp02aa1n03x5               g193(.a(new_n288), .b(new_n281), .o1(\s[28] ));
  norb02aa1n02x5               g194(.a(new_n275), .b(new_n280), .out0(new_n290));
  oaih12aa1n02x5               g195(.a(new_n290), .b(new_n286), .c(new_n283), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n278), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[28] ), .b(\a[29] ), .out0(new_n293));
  tech160nm_fiaoi012aa1n02p5x5 g198(.a(new_n293), .b(new_n291), .c(new_n292), .o1(new_n294));
  aobi12aa1n02x7               g199(.a(new_n290), .b(new_n274), .c(new_n272), .out0(new_n295));
  nano22aa1n02x4               g200(.a(new_n295), .b(new_n292), .c(new_n293), .out0(new_n296));
  norp02aa1n03x5               g201(.a(new_n294), .b(new_n296), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g203(.a(new_n275), .b(new_n293), .c(new_n280), .out0(new_n299));
  oaih12aa1n02x5               g204(.a(new_n299), .b(new_n286), .c(new_n283), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[29] ), .b(\a[30] ), .out0(new_n302));
  tech160nm_fiaoi012aa1n02p5x5 g207(.a(new_n302), .b(new_n300), .c(new_n301), .o1(new_n303));
  aobi12aa1n02x7               g208(.a(new_n299), .b(new_n274), .c(new_n272), .out0(new_n304));
  nano22aa1n03x5               g209(.a(new_n304), .b(new_n301), .c(new_n302), .out0(new_n305));
  norp02aa1n03x5               g210(.a(new_n303), .b(new_n305), .o1(\s[30] ));
  norb02aa1n02x5               g211(.a(new_n299), .b(new_n302), .out0(new_n307));
  aobi12aa1n02x7               g212(.a(new_n307), .b(new_n274), .c(new_n272), .out0(new_n308));
  oao003aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n309));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  nano22aa1n02x4               g215(.a(new_n308), .b(new_n309), .c(new_n310), .out0(new_n311));
  oaih12aa1n02x5               g216(.a(new_n307), .b(new_n286), .c(new_n283), .o1(new_n312));
  tech160nm_fiaoi012aa1n02p5x5 g217(.a(new_n310), .b(new_n312), .c(new_n309), .o1(new_n313));
  norp02aa1n03x5               g218(.a(new_n313), .b(new_n311), .o1(\s[31] ));
  xnrb03aa1n02x5               g219(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g220(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g222(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoib12aa1n02x5               g223(.a(new_n121), .b(new_n109), .c(new_n116), .out0(new_n319));
  xnrb03aa1n02x5               g224(.a(new_n319), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g225(.a(new_n158), .b(new_n115), .c(new_n319), .out0(new_n321));
  xnrb03aa1n02x5               g226(.a(new_n321), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g227(.a(\a[7] ), .b(\b[6] ), .c(new_n321), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g229(.a(new_n125), .b(new_n160), .c(new_n157), .out0(\s[9] ));
endmodule

