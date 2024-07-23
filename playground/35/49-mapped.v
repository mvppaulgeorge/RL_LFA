// Benchmark "adder" written by ABC on Thu Jul 18 06:21:17 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n213, new_n214,
    new_n215, new_n216, new_n217, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n229, new_n230,
    new_n231, new_n232, new_n233, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n247, new_n248, new_n249, new_n250, new_n251, new_n252, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n304, new_n305, new_n308, new_n310, new_n312, new_n314;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand42aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n06x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nor042aa1n06x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oaih12aa1n06x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  norp02aa1n12x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nor022aa1n16x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nand02aa1n10x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  tech160nm_fiaoi012aa1n03p5x5 g012(.a(new_n105), .b(new_n103), .c(new_n106), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nand42aa1n08x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand42aa1n06x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nanb03aa1d24x5               g017(.a(new_n111), .b(new_n112), .c(new_n110), .out0(new_n113));
  xorc02aa1n12x5               g018(.a(\a[8] ), .b(\b[7] ), .out0(new_n114));
  inv040aa1d32x5               g019(.a(\a[5] ), .o1(new_n115));
  inv040aa1d28x5               g020(.a(\b[4] ), .o1(new_n116));
  nand42aa1n06x5               g021(.a(new_n116), .b(new_n115), .o1(new_n117));
  nand42aa1n08x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  oai112aa1n06x5               g023(.a(new_n117), .b(new_n118), .c(\b[5] ), .d(\a[6] ), .o1(new_n119));
  norb03aa1n12x5               g024(.a(new_n114), .b(new_n113), .c(new_n119), .out0(new_n120));
  tech160nm_fixnrc02aa1n05x5   g025(.a(\b[7] ), .b(\a[8] ), .out0(new_n121));
  oa0022aa1n06x5               g026(.a(\a[6] ), .b(\b[5] ), .c(\a[5] ), .d(\b[4] ), .o(new_n122));
  aob012aa1n03x5               g027(.a(new_n111), .b(\b[7] ), .c(\a[8] ), .out0(new_n123));
  oa0012aa1n06x5               g028(.a(new_n123), .b(\b[7] ), .c(\a[8] ), .o(new_n124));
  oai013aa1d12x5               g029(.a(new_n124), .b(new_n113), .c(new_n121), .d(new_n122), .o1(new_n125));
  nanp02aa1n12x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n97), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n125), .c(new_n109), .d(new_n120), .o1(new_n128));
  nor042aa1n09x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand42aa1n20x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n128), .c(new_n98), .out0(\s[10] ));
  nanp03aa1n03x5               g037(.a(new_n128), .b(new_n98), .c(new_n131), .o1(new_n133));
  xnrc02aa1n12x5               g038(.a(\b[10] ), .b(\a[11] ), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n133), .c(new_n130), .out0(\s[11] ));
  inv000aa1d42x5               g040(.a(\a[12] ), .o1(new_n136));
  nor002aa1n03x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  tech160nm_fixorc02aa1n03p5x5 g042(.a(\a[11] ), .b(\b[10] ), .out0(new_n138));
  aoi013aa1n03x5               g043(.a(new_n137), .b(new_n133), .c(new_n130), .d(new_n138), .o1(new_n139));
  xorb03aa1n02x5               g044(.a(new_n139), .b(\b[11] ), .c(new_n136), .out0(\s[12] ));
  xnrc02aa1n12x5               g045(.a(\b[11] ), .b(\a[12] ), .out0(new_n141));
  nano23aa1n03x7               g046(.a(new_n97), .b(new_n129), .c(new_n130), .d(new_n126), .out0(new_n142));
  nano22aa1n02x4               g047(.a(new_n141), .b(new_n142), .c(new_n138), .out0(new_n143));
  aoai13aa1n06x5               g048(.a(new_n143), .b(new_n125), .c(new_n109), .d(new_n120), .o1(new_n144));
  oai012aa1d24x5               g049(.a(new_n130), .b(new_n129), .c(new_n97), .o1(new_n145));
  inv000aa1d42x5               g050(.a(\b[11] ), .o1(new_n146));
  tech160nm_fioaoi03aa1n03p5x5 g051(.a(new_n136), .b(new_n146), .c(new_n137), .o1(new_n147));
  oai013aa1d12x5               g052(.a(new_n147), .b(new_n141), .c(new_n134), .d(new_n145), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  xorc02aa1n12x5               g054(.a(\a[13] ), .b(\b[12] ), .out0(new_n150));
  xnbna2aa1n03x5               g055(.a(new_n150), .b(new_n144), .c(new_n149), .out0(\s[13] ));
  nor042aa1d18x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  inv000aa1n06x5               g057(.a(new_n152), .o1(new_n153));
  aob012aa1n02x5               g058(.a(new_n150), .b(new_n144), .c(new_n149), .out0(new_n154));
  xorc02aa1n12x5               g059(.a(\a[14] ), .b(\b[13] ), .out0(new_n155));
  xnbna2aa1n03x5               g060(.a(new_n155), .b(new_n154), .c(new_n153), .out0(\s[14] ));
  nanp02aa1n02x5               g061(.a(new_n155), .b(new_n150), .o1(new_n157));
  oao003aa1n03x5               g062(.a(\a[14] ), .b(\b[13] ), .c(new_n153), .carry(new_n158));
  aoai13aa1n06x5               g063(.a(new_n158), .b(new_n157), .c(new_n144), .d(new_n149), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n20x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  nand42aa1d28x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  nor002aa1n16x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  nanp02aa1n12x5               g069(.a(\b[15] ), .b(\a[16] ), .o1(new_n165));
  aoi122aa1n02x5               g070(.a(new_n161), .b(new_n164), .c(new_n165), .d(new_n159), .e(new_n162), .o1(new_n166));
  aoi012aa1n06x5               g071(.a(new_n161), .b(new_n159), .c(new_n162), .o1(new_n167));
  nano22aa1n03x7               g072(.a(new_n167), .b(new_n164), .c(new_n165), .out0(new_n168));
  norp02aa1n02x5               g073(.a(new_n168), .b(new_n166), .o1(\s[16] ));
  inv040aa1n02x5               g074(.a(new_n141), .o1(new_n170));
  nano23aa1n06x5               g075(.a(new_n161), .b(new_n163), .c(new_n165), .d(new_n162), .out0(new_n171));
  nand03aa1n04x5               g076(.a(new_n171), .b(new_n150), .c(new_n155), .o1(new_n172));
  nano32aa1n03x7               g077(.a(new_n172), .b(new_n142), .c(new_n170), .d(new_n138), .out0(new_n173));
  aoai13aa1n12x5               g078(.a(new_n173), .b(new_n125), .c(new_n109), .d(new_n120), .o1(new_n174));
  inv000aa1d42x5               g079(.a(new_n161), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(new_n165), .b(new_n162), .o1(new_n176));
  aoai13aa1n06x5               g081(.a(new_n164), .b(new_n176), .c(new_n158), .d(new_n175), .o1(new_n177));
  aoib12aa1n12x5               g082(.a(new_n177), .b(new_n148), .c(new_n172), .out0(new_n178));
  nanp02aa1n12x5               g083(.a(new_n174), .b(new_n178), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g085(.a(\a[18] ), .o1(new_n181));
  inv000aa1d42x5               g086(.a(\a[17] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(\b[16] ), .o1(new_n183));
  oaoi03aa1n02x5               g088(.a(new_n182), .b(new_n183), .c(new_n179), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(new_n181), .out0(\s[18] ));
  xroi22aa1d06x4               g090(.a(new_n182), .b(\b[16] ), .c(new_n181), .d(\b[17] ), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(new_n183), .b(new_n182), .o1(new_n187));
  oaoi03aa1n02x5               g092(.a(\a[18] ), .b(\b[17] ), .c(new_n187), .o1(new_n188));
  nor002aa1d32x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  nanp02aa1n06x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n188), .c(new_n179), .d(new_n186), .o1(new_n192));
  aoi112aa1n02x5               g097(.a(new_n191), .b(new_n188), .c(new_n179), .d(new_n186), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n192), .b(new_n193), .out0(\s[19] ));
  xnrc02aa1n02x5               g099(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n06x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  nand22aa1n09x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  nona22aa1n02x5               g103(.a(new_n192), .b(new_n198), .c(new_n189), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n189), .o1(new_n200));
  aobi12aa1n06x5               g105(.a(new_n198), .b(new_n192), .c(new_n200), .out0(new_n201));
  norb02aa1n03x4               g106(.a(new_n199), .b(new_n201), .out0(\s[20] ));
  nano23aa1n09x5               g107(.a(new_n189), .b(new_n196), .c(new_n197), .d(new_n190), .out0(new_n203));
  nanp02aa1n02x5               g108(.a(new_n186), .b(new_n203), .o1(new_n204));
  oaih22aa1n04x5               g109(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n205));
  oaib12aa1n06x5               g110(.a(new_n205), .b(new_n181), .c(\b[17] ), .out0(new_n206));
  nona23aa1d18x5               g111(.a(new_n197), .b(new_n190), .c(new_n189), .d(new_n196), .out0(new_n207));
  aoi012aa1n06x5               g112(.a(new_n196), .b(new_n189), .c(new_n197), .o1(new_n208));
  oai012aa1d24x5               g113(.a(new_n208), .b(new_n207), .c(new_n206), .o1(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  aoai13aa1n06x5               g115(.a(new_n210), .b(new_n204), .c(new_n174), .d(new_n178), .o1(new_n211));
  xorb03aa1n02x5               g116(.a(new_n211), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g117(.a(\b[20] ), .b(\a[21] ), .o1(new_n213));
  xorc02aa1n02x5               g118(.a(\a[21] ), .b(\b[20] ), .out0(new_n214));
  xorc02aa1n02x5               g119(.a(\a[22] ), .b(\b[21] ), .out0(new_n215));
  aoi112aa1n02x5               g120(.a(new_n213), .b(new_n215), .c(new_n211), .d(new_n214), .o1(new_n216));
  aoai13aa1n03x5               g121(.a(new_n215), .b(new_n213), .c(new_n211), .d(new_n214), .o1(new_n217));
  norb02aa1n02x5               g122(.a(new_n217), .b(new_n216), .out0(\s[22] ));
  inv000aa1d42x5               g123(.a(\a[21] ), .o1(new_n219));
  inv040aa1d32x5               g124(.a(\a[22] ), .o1(new_n220));
  xroi22aa1d06x4               g125(.a(new_n219), .b(\b[20] ), .c(new_n220), .d(\b[21] ), .out0(new_n221));
  nanp03aa1n03x5               g126(.a(new_n221), .b(new_n186), .c(new_n203), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[21] ), .o1(new_n223));
  oaoi03aa1n12x5               g128(.a(new_n220), .b(new_n223), .c(new_n213), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  aoi012aa1n02x5               g130(.a(new_n225), .b(new_n209), .c(new_n221), .o1(new_n226));
  aoai13aa1n04x5               g131(.a(new_n226), .b(new_n222), .c(new_n174), .d(new_n178), .o1(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g133(.a(\b[22] ), .b(\a[23] ), .o1(new_n229));
  tech160nm_fixorc02aa1n05x5   g134(.a(\a[23] ), .b(\b[22] ), .out0(new_n230));
  tech160nm_fixorc02aa1n02p5x5 g135(.a(\a[24] ), .b(\b[23] ), .out0(new_n231));
  aoi112aa1n02x5               g136(.a(new_n229), .b(new_n231), .c(new_n227), .d(new_n230), .o1(new_n232));
  aoai13aa1n03x5               g137(.a(new_n231), .b(new_n229), .c(new_n227), .d(new_n230), .o1(new_n233));
  norb02aa1n03x4               g138(.a(new_n233), .b(new_n232), .out0(\s[24] ));
  and002aa1n06x5               g139(.a(new_n231), .b(new_n230), .o(new_n235));
  inv000aa1n02x5               g140(.a(new_n235), .o1(new_n236));
  nano32aa1n02x4               g141(.a(new_n236), .b(new_n221), .c(new_n186), .d(new_n203), .out0(new_n237));
  inv030aa1n04x5               g142(.a(new_n208), .o1(new_n238));
  aoai13aa1n06x5               g143(.a(new_n221), .b(new_n238), .c(new_n203), .d(new_n188), .o1(new_n239));
  aoi112aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n240));
  oab012aa1n02x4               g145(.a(new_n240), .b(\a[24] ), .c(\b[23] ), .out0(new_n241));
  aoai13aa1n12x5               g146(.a(new_n241), .b(new_n236), .c(new_n239), .d(new_n224), .o1(new_n242));
  tech160nm_fiaoi012aa1n05x5   g147(.a(new_n242), .b(new_n179), .c(new_n237), .o1(new_n243));
  xnrc02aa1n12x5               g148(.a(\b[24] ), .b(\a[25] ), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  xnrc02aa1n02x5               g150(.a(new_n243), .b(new_n245), .out0(\s[25] ));
  nor042aa1n03x5               g151(.a(\b[24] ), .b(\a[25] ), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  aoai13aa1n06x5               g153(.a(new_n245), .b(new_n242), .c(new_n179), .d(new_n237), .o1(new_n249));
  xnrc02aa1n12x5               g154(.a(\b[25] ), .b(\a[26] ), .out0(new_n250));
  nand43aa1n03x5               g155(.a(new_n249), .b(new_n248), .c(new_n250), .o1(new_n251));
  tech160nm_fiaoi012aa1n03p5x5 g156(.a(new_n250), .b(new_n249), .c(new_n248), .o1(new_n252));
  norb02aa1n03x4               g157(.a(new_n251), .b(new_n252), .out0(\s[26] ));
  norp02aa1n24x5               g158(.a(new_n250), .b(new_n244), .o1(new_n254));
  nano22aa1n03x7               g159(.a(new_n222), .b(new_n235), .c(new_n254), .out0(new_n255));
  nand22aa1n09x5               g160(.a(new_n179), .b(new_n255), .o1(new_n256));
  oao003aa1n02x5               g161(.a(\a[26] ), .b(\b[25] ), .c(new_n248), .carry(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  aoi012aa1d18x5               g163(.a(new_n258), .b(new_n242), .c(new_n254), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[27] ), .b(\b[26] ), .out0(new_n260));
  xnbna2aa1n06x5               g165(.a(new_n260), .b(new_n256), .c(new_n259), .out0(\s[27] ));
  nor042aa1n03x5               g166(.a(\b[26] ), .b(\a[27] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n260), .o1(new_n264));
  aoi012aa1n06x5               g169(.a(new_n264), .b(new_n256), .c(new_n259), .o1(new_n265));
  xnrc02aa1n12x5               g170(.a(\b[27] ), .b(\a[28] ), .out0(new_n266));
  nano22aa1n03x5               g171(.a(new_n265), .b(new_n263), .c(new_n266), .out0(new_n267));
  aobi12aa1n06x5               g172(.a(new_n255), .b(new_n174), .c(new_n178), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n235), .b(new_n225), .c(new_n209), .d(new_n221), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n254), .o1(new_n270));
  aoai13aa1n06x5               g175(.a(new_n257), .b(new_n270), .c(new_n269), .d(new_n241), .o1(new_n271));
  oaih12aa1n02x5               g176(.a(new_n260), .b(new_n271), .c(new_n268), .o1(new_n272));
  aoi012aa1n03x5               g177(.a(new_n266), .b(new_n272), .c(new_n263), .o1(new_n273));
  norp02aa1n03x5               g178(.a(new_n273), .b(new_n267), .o1(\s[28] ));
  norb02aa1d21x5               g179(.a(new_n260), .b(new_n266), .out0(new_n275));
  tech160nm_fioai012aa1n03p5x5 g180(.a(new_n275), .b(new_n271), .c(new_n268), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[28] ), .b(\b[27] ), .c(new_n263), .carry(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[28] ), .b(\a[29] ), .out0(new_n278));
  aoi012aa1n03x5               g183(.a(new_n278), .b(new_n276), .c(new_n277), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n275), .o1(new_n280));
  aoi012aa1n06x5               g185(.a(new_n280), .b(new_n256), .c(new_n259), .o1(new_n281));
  nano22aa1n03x5               g186(.a(new_n281), .b(new_n277), .c(new_n278), .out0(new_n282));
  norp02aa1n03x5               g187(.a(new_n279), .b(new_n282), .o1(\s[29] ));
  xorb03aa1n02x5               g188(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g189(.a(new_n260), .b(new_n278), .c(new_n266), .out0(new_n285));
  oaih12aa1n02x5               g190(.a(new_n285), .b(new_n271), .c(new_n268), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[29] ), .b(\b[28] ), .c(new_n277), .carry(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[29] ), .b(\a[30] ), .out0(new_n288));
  aoi012aa1n02x7               g193(.a(new_n288), .b(new_n286), .c(new_n287), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n285), .o1(new_n290));
  tech160nm_fiaoi012aa1n05x5   g195(.a(new_n290), .b(new_n256), .c(new_n259), .o1(new_n291));
  nano22aa1n03x5               g196(.a(new_n291), .b(new_n287), .c(new_n288), .out0(new_n292));
  norp02aa1n03x5               g197(.a(new_n289), .b(new_n292), .o1(\s[30] ));
  xnrc02aa1n02x5               g198(.a(\b[30] ), .b(\a[31] ), .out0(new_n294));
  norb02aa1n02x5               g199(.a(new_n285), .b(new_n288), .out0(new_n295));
  oaih12aa1n02x5               g200(.a(new_n295), .b(new_n271), .c(new_n268), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[30] ), .b(\b[29] ), .c(new_n287), .carry(new_n297));
  aoi012aa1n03x5               g202(.a(new_n294), .b(new_n296), .c(new_n297), .o1(new_n298));
  inv000aa1n02x5               g203(.a(new_n295), .o1(new_n299));
  aoi012aa1n06x5               g204(.a(new_n299), .b(new_n256), .c(new_n259), .o1(new_n300));
  nano22aa1n03x5               g205(.a(new_n300), .b(new_n294), .c(new_n297), .out0(new_n301));
  norp02aa1n03x5               g206(.a(new_n298), .b(new_n301), .o1(\s[31] ));
  xnrb03aa1n02x5               g207(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nano22aa1n02x4               g208(.a(new_n103), .b(new_n102), .c(new_n104), .out0(new_n304));
  oaib12aa1n02x5               g209(.a(new_n104), .b(new_n105), .c(new_n106), .out0(new_n305));
  obai22aa1n02x7               g210(.a(new_n106), .b(new_n109), .c(new_n304), .d(new_n305), .out0(\s[4] ));
  xorb03aa1n02x5               g211(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g212(.a(new_n115), .b(new_n116), .c(new_n109), .o1(new_n308));
  xnrb03aa1n02x5               g213(.a(new_n308), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g214(.a(\a[6] ), .b(\b[5] ), .c(new_n308), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oai012aa1n02x5               g216(.a(new_n110), .b(new_n310), .c(new_n111), .o1(new_n312));
  xnrc02aa1n02x5               g217(.a(new_n312), .b(new_n114), .out0(\s[8] ));
  aoi112aa1n02x5               g218(.a(new_n125), .b(new_n127), .c(new_n109), .d(new_n120), .o1(new_n314));
  norb02aa1n02x5               g219(.a(new_n128), .b(new_n314), .out0(\s[9] ));
endmodule


