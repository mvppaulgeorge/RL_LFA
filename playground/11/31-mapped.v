// Benchmark "adder" written by ABC on Wed Jul 17 17:51:41 2024

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
    new_n140, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n230, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n290,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n327, new_n329, new_n331,
    new_n332, new_n333, new_n334, new_n335, new_n338, new_n339, new_n340,
    new_n342, new_n344, new_n345, new_n346, new_n347, new_n348;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  xnrc02aa1n02x5               g004(.a(\b[2] ), .b(\a[3] ), .out0(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nor042aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand02aa1d06x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  tech160nm_fioai012aa1n05x5   g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  oa0022aa1n02x5               g009(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n105));
  tech160nm_fioai012aa1n04x5   g010(.a(new_n105), .b(new_n100), .c(new_n104), .o1(new_n106));
  inv040aa1d32x5               g011(.a(\a[6] ), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\b[5] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(new_n108), .b(new_n107), .o1(new_n109));
  inv040aa1d32x5               g014(.a(\a[7] ), .o1(new_n110));
  inv000aa1d42x5               g015(.a(\b[6] ), .o1(new_n111));
  nand42aa1n16x5               g016(.a(new_n111), .b(new_n110), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand02aa1n03x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nand03aa1n02x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  nor042aa1n06x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  aoi012aa1n02x5               g021(.a(new_n116), .b(\a[4] ), .c(\b[3] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  inv040aa1d32x5               g023(.a(\a[8] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[7] ), .o1(new_n120));
  tech160nm_finand02aa1n03p5x5 g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[7] ), .b(\a[8] ), .o1(new_n122));
  nanp03aa1n02x5               g027(.a(new_n121), .b(new_n118), .c(new_n122), .o1(new_n123));
  nano23aa1n06x5               g028(.a(new_n115), .b(new_n123), .c(new_n117), .d(new_n109), .out0(new_n124));
  tech160nm_fioaoi03aa1n03p5x5 g029(.a(new_n107), .b(new_n108), .c(new_n116), .o1(new_n125));
  aoi022aa1d24x5               g030(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  aoai13aa1n09x5               g032(.a(new_n121), .b(new_n127), .c(new_n125), .d(new_n112), .o1(new_n128));
  xorc02aa1n02x5               g033(.a(\a[9] ), .b(\b[8] ), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n128), .c(new_n106), .d(new_n124), .o1(new_n130));
  nor042aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nanp02aa1n12x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  norb02aa1n02x7               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n130), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g039(.a(\a[10] ), .o1(new_n135));
  xroi22aa1d04x5               g040(.a(new_n135), .b(\b[9] ), .c(new_n98), .d(\a[9] ), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n136), .b(new_n128), .c(new_n106), .d(new_n124), .o1(new_n137));
  aoi013aa1n06x4               g042(.a(new_n131), .b(new_n132), .c(new_n97), .d(new_n98), .o1(new_n138));
  xnrc02aa1n12x5               g043(.a(\b[10] ), .b(\a[11] ), .out0(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n140), .b(new_n137), .c(new_n138), .out0(\s[11] ));
  nor042aa1n03x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  inv040aa1n03x5               g047(.a(new_n142), .o1(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n139), .c(new_n137), .d(new_n138), .o1(new_n144));
  xorb03aa1n02x5               g049(.a(new_n144), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  tech160nm_fixnrc02aa1n05x5   g050(.a(\b[11] ), .b(\a[12] ), .out0(new_n146));
  nano23aa1n02x4               g051(.a(new_n146), .b(new_n139), .c(new_n129), .d(new_n133), .out0(new_n147));
  aoai13aa1n06x5               g052(.a(new_n147), .b(new_n128), .c(new_n106), .d(new_n124), .o1(new_n148));
  oaoi03aa1n02x5               g053(.a(\a[12] ), .b(\b[11] ), .c(new_n143), .o1(new_n149));
  inv000aa1n02x5               g054(.a(new_n149), .o1(new_n150));
  oai013aa1d12x5               g055(.a(new_n150), .b(new_n138), .c(new_n139), .d(new_n146), .o1(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  nor042aa1n06x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nand42aa1n08x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanb02aa1n02x5               g059(.a(new_n153), .b(new_n154), .out0(new_n155));
  xobna2aa1n03x5               g060(.a(new_n155), .b(new_n148), .c(new_n152), .out0(\s[13] ));
  nanp02aa1n02x5               g061(.a(new_n148), .b(new_n152), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n153), .b(new_n157), .c(new_n154), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nanp02aa1n02x5               g064(.a(new_n124), .b(new_n106), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(new_n125), .b(new_n112), .o1(new_n161));
  aoi022aa1n02x5               g066(.a(new_n161), .b(new_n126), .c(new_n120), .d(new_n119), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(new_n162), .b(new_n160), .o1(new_n163));
  nor022aa1n04x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand02aa1n06x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nano23aa1n03x5               g070(.a(new_n153), .b(new_n164), .c(new_n165), .d(new_n154), .out0(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n151), .c(new_n163), .d(new_n147), .o1(new_n167));
  aoi012aa1n02x5               g072(.a(new_n164), .b(new_n153), .c(new_n165), .o1(new_n168));
  nor042aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanp02aa1n12x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  norb02aa1n02x7               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n167), .c(new_n168), .out0(\s[15] ));
  nona23aa1n02x4               g077(.a(new_n165), .b(new_n154), .c(new_n153), .d(new_n164), .out0(new_n173));
  aoai13aa1n06x5               g078(.a(new_n168), .b(new_n173), .c(new_n148), .d(new_n152), .o1(new_n174));
  tech160nm_fixnrc02aa1n02p5x5 g079(.a(\b[15] ), .b(\a[16] ), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n169), .c(new_n174), .d(new_n170), .o1(new_n176));
  aoi112aa1n02x5               g081(.a(new_n169), .b(new_n175), .c(new_n174), .d(new_n170), .o1(new_n177));
  nanb02aa1n03x5               g082(.a(new_n177), .b(new_n176), .out0(\s[16] ));
  xorc02aa1n02x5               g083(.a(\a[17] ), .b(\b[16] ), .out0(new_n179));
  nor042aa1n02x5               g084(.a(new_n146), .b(new_n139), .o1(new_n180));
  nanb03aa1n03x5               g085(.a(new_n175), .b(new_n166), .c(new_n171), .out0(new_n181));
  nano22aa1n03x7               g086(.a(new_n181), .b(new_n136), .c(new_n180), .out0(new_n182));
  aoai13aa1n06x5               g087(.a(new_n182), .b(new_n128), .c(new_n106), .d(new_n124), .o1(new_n183));
  norp02aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  norb03aa1n03x5               g089(.a(new_n171), .b(new_n173), .c(new_n175), .out0(new_n185));
  aob012aa1n02x5               g090(.a(new_n170), .b(\b[15] ), .c(\a[16] ), .out0(new_n186));
  oaoi13aa1n06x5               g091(.a(new_n186), .b(new_n168), .c(\a[15] ), .d(\b[14] ), .o1(new_n187));
  aoi112aa1n09x5               g092(.a(new_n184), .b(new_n187), .c(new_n151), .d(new_n185), .o1(new_n188));
  xnbna2aa1n03x5               g093(.a(new_n179), .b(new_n183), .c(new_n188), .out0(\s[17] ));
  inv040aa1d32x5               g094(.a(\a[17] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[16] ), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(new_n191), .b(new_n190), .o1(new_n192));
  aoi012aa1n12x5               g097(.a(new_n128), .b(new_n106), .c(new_n124), .o1(new_n193));
  oaib12aa1n18x5               g098(.a(new_n188), .b(new_n193), .c(new_n182), .out0(new_n194));
  oaib12aa1n06x5               g099(.a(new_n194), .b(new_n191), .c(\a[17] ), .out0(new_n195));
  norp02aa1n02x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(\b[17] ), .b(\a[18] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n195), .c(new_n192), .out0(\s[18] ));
  inv040aa1d32x5               g104(.a(\a[18] ), .o1(new_n200));
  xroi22aa1d06x4               g105(.a(new_n190), .b(\b[16] ), .c(new_n200), .d(\b[17] ), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n201), .o1(new_n202));
  aoi013aa1n06x4               g107(.a(new_n196), .b(new_n197), .c(new_n190), .d(new_n191), .o1(new_n203));
  aoai13aa1n03x5               g108(.a(new_n203), .b(new_n202), .c(new_n183), .d(new_n188), .o1(new_n204));
  nor022aa1n08x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nand22aa1n04x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  oaoi03aa1n03x5               g112(.a(\a[18] ), .b(\b[17] ), .c(new_n192), .o1(new_n208));
  aoi112aa1n03x4               g113(.a(new_n207), .b(new_n208), .c(new_n194), .d(new_n201), .o1(new_n209));
  tech160nm_fiaoi012aa1n02p5x5 g114(.a(new_n209), .b(new_n204), .c(new_n207), .o1(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nand02aa1d06x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nanb02aa1n02x5               g118(.a(new_n212), .b(new_n213), .out0(new_n214));
  aoai13aa1n02x7               g119(.a(new_n214), .b(new_n205), .c(new_n204), .d(new_n207), .o1(new_n215));
  aoai13aa1n06x5               g120(.a(new_n207), .b(new_n208), .c(new_n194), .d(new_n201), .o1(new_n216));
  nona22aa1n03x5               g121(.a(new_n216), .b(new_n214), .c(new_n205), .out0(new_n217));
  nanp02aa1n03x5               g122(.a(new_n215), .b(new_n217), .o1(\s[20] ));
  nona23aa1n09x5               g123(.a(new_n213), .b(new_n206), .c(new_n205), .d(new_n212), .out0(new_n219));
  nano22aa1n09x5               g124(.a(new_n219), .b(new_n179), .c(new_n198), .out0(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  oa0012aa1n06x5               g126(.a(new_n213), .b(new_n212), .c(new_n205), .o(new_n222));
  oabi12aa1n18x5               g127(.a(new_n222), .b(new_n203), .c(new_n219), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n221), .c(new_n183), .d(new_n188), .o1(new_n225));
  xnrc02aa1n12x5               g130(.a(\b[20] ), .b(\a[21] ), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  aoi112aa1n02x7               g132(.a(new_n227), .b(new_n223), .c(new_n194), .d(new_n220), .o1(new_n228));
  tech160nm_fiaoi012aa1n02p5x5 g133(.a(new_n228), .b(new_n225), .c(new_n227), .o1(\s[21] ));
  nor042aa1n02x5               g134(.a(\b[20] ), .b(\a[21] ), .o1(new_n230));
  tech160nm_fixnrc02aa1n04x5   g135(.a(\b[21] ), .b(\a[22] ), .out0(new_n231));
  aoai13aa1n03x5               g136(.a(new_n231), .b(new_n230), .c(new_n225), .d(new_n227), .o1(new_n232));
  aoai13aa1n06x5               g137(.a(new_n227), .b(new_n223), .c(new_n194), .d(new_n220), .o1(new_n233));
  nona22aa1n03x5               g138(.a(new_n233), .b(new_n231), .c(new_n230), .out0(new_n234));
  nanp02aa1n03x5               g139(.a(new_n232), .b(new_n234), .o1(\s[22] ));
  nano23aa1n06x5               g140(.a(new_n205), .b(new_n212), .c(new_n213), .d(new_n206), .out0(new_n236));
  norp02aa1n12x5               g141(.a(new_n231), .b(new_n226), .o1(new_n237));
  nand23aa1n06x5               g142(.a(new_n201), .b(new_n237), .c(new_n236), .o1(new_n238));
  inv000aa1d42x5               g143(.a(\a[22] ), .o1(new_n239));
  inv000aa1d42x5               g144(.a(\b[21] ), .o1(new_n240));
  oaoi03aa1n09x5               g145(.a(new_n239), .b(new_n240), .c(new_n230), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  aoi012aa1n02x5               g147(.a(new_n242), .b(new_n223), .c(new_n237), .o1(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n238), .c(new_n183), .d(new_n188), .o1(new_n244));
  xorb03aa1n02x5               g149(.a(new_n244), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  tech160nm_fixorc02aa1n05x5   g151(.a(\a[23] ), .b(\b[22] ), .out0(new_n247));
  nor042aa1n02x5               g152(.a(\b[23] ), .b(\a[24] ), .o1(new_n248));
  nanp02aa1n02x5               g153(.a(\b[23] ), .b(\a[24] ), .o1(new_n249));
  nanb02aa1n03x5               g154(.a(new_n248), .b(new_n249), .out0(new_n250));
  aoai13aa1n03x5               g155(.a(new_n250), .b(new_n246), .c(new_n244), .d(new_n247), .o1(new_n251));
  nand42aa1n02x5               g156(.a(new_n244), .b(new_n247), .o1(new_n252));
  nona22aa1n03x5               g157(.a(new_n252), .b(new_n250), .c(new_n246), .out0(new_n253));
  nanp02aa1n03x5               g158(.a(new_n253), .b(new_n251), .o1(\s[24] ));
  norp03aa1n02x5               g159(.a(new_n138), .b(new_n139), .c(new_n146), .o1(new_n255));
  oai012aa1n02x5               g160(.a(new_n185), .b(new_n255), .c(new_n149), .o1(new_n256));
  nona22aa1n02x4               g161(.a(new_n256), .b(new_n187), .c(new_n184), .out0(new_n257));
  norb02aa1n02x7               g162(.a(new_n247), .b(new_n250), .out0(new_n258));
  inv030aa1n02x5               g163(.a(new_n258), .o1(new_n259));
  nano32aa1n03x7               g164(.a(new_n259), .b(new_n201), .c(new_n237), .d(new_n236), .out0(new_n260));
  aoai13aa1n02x5               g165(.a(new_n260), .b(new_n257), .c(new_n163), .d(new_n182), .o1(new_n261));
  inv000aa1n02x5               g166(.a(new_n260), .o1(new_n262));
  aoai13aa1n06x5               g167(.a(new_n237), .b(new_n222), .c(new_n236), .d(new_n208), .o1(new_n263));
  oai012aa1n02x7               g168(.a(new_n249), .b(new_n248), .c(new_n246), .o1(new_n264));
  aoai13aa1n12x5               g169(.a(new_n264), .b(new_n259), .c(new_n263), .d(new_n241), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  aoai13aa1n03x5               g171(.a(new_n266), .b(new_n262), .c(new_n183), .d(new_n188), .o1(new_n267));
  xorc02aa1n12x5               g172(.a(\a[25] ), .b(\b[24] ), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n258), .b(new_n242), .c(new_n223), .d(new_n237), .o1(new_n269));
  nano22aa1n02x4               g174(.a(new_n268), .b(new_n269), .c(new_n264), .out0(new_n270));
  aoi022aa1n02x5               g175(.a(new_n267), .b(new_n268), .c(new_n261), .d(new_n270), .o1(\s[25] ));
  norp02aa1n02x5               g176(.a(\b[24] ), .b(\a[25] ), .o1(new_n272));
  tech160nm_fixnrc02aa1n04x5   g177(.a(\b[25] ), .b(\a[26] ), .out0(new_n273));
  aoai13aa1n02x7               g178(.a(new_n273), .b(new_n272), .c(new_n267), .d(new_n268), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n268), .b(new_n265), .c(new_n194), .d(new_n260), .o1(new_n275));
  nona22aa1n03x5               g180(.a(new_n275), .b(new_n273), .c(new_n272), .out0(new_n276));
  nanp02aa1n03x5               g181(.a(new_n274), .b(new_n276), .o1(\s[26] ));
  norb02aa1n03x5               g182(.a(new_n268), .b(new_n273), .out0(new_n278));
  nano22aa1n03x7               g183(.a(new_n238), .b(new_n258), .c(new_n278), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n257), .c(new_n163), .d(new_n182), .o1(new_n280));
  nor042aa1n09x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  and002aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o(new_n282));
  nor022aa1n04x5               g187(.a(new_n282), .b(new_n281), .o1(new_n283));
  nanp02aa1n02x5               g188(.a(\b[25] ), .b(\a[26] ), .o1(new_n284));
  oai022aa1n02x5               g189(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n285));
  aoi122aa1n02x7               g190(.a(new_n283), .b(new_n285), .c(new_n284), .d(new_n265), .e(new_n278), .o1(new_n286));
  aoi022aa1n09x5               g191(.a(new_n265), .b(new_n278), .c(new_n284), .d(new_n285), .o1(new_n287));
  nand42aa1n02x5               g192(.a(new_n287), .b(new_n280), .o1(new_n288));
  aoi022aa1n02x7               g193(.a(new_n288), .b(new_n283), .c(new_n286), .d(new_n280), .o1(\s[27] ));
  aobi12aa1n02x5               g194(.a(new_n279), .b(new_n183), .c(new_n188), .out0(new_n290));
  inv000aa1d42x5               g195(.a(new_n278), .o1(new_n291));
  nanp02aa1n02x5               g196(.a(new_n285), .b(new_n284), .o1(new_n292));
  aoai13aa1n04x5               g197(.a(new_n292), .b(new_n291), .c(new_n269), .d(new_n264), .o1(new_n293));
  oabi12aa1n02x5               g198(.a(new_n282), .b(new_n293), .c(new_n290), .out0(new_n294));
  inv000aa1n06x5               g199(.a(new_n281), .o1(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n282), .c(new_n287), .d(new_n280), .o1(new_n296));
  xorc02aa1n02x5               g201(.a(\a[28] ), .b(\b[27] ), .out0(new_n297));
  norp02aa1n02x5               g202(.a(new_n297), .b(new_n281), .o1(new_n298));
  aoi022aa1n03x5               g203(.a(new_n296), .b(new_n297), .c(new_n294), .d(new_n298), .o1(\s[28] ));
  and002aa1n06x5               g204(.a(new_n297), .b(new_n283), .o(new_n300));
  aoai13aa1n02x7               g205(.a(new_n300), .b(new_n293), .c(new_n194), .d(new_n279), .o1(new_n301));
  inv000aa1n02x5               g206(.a(new_n300), .o1(new_n302));
  oao003aa1n03x5               g207(.a(\a[28] ), .b(\b[27] ), .c(new_n295), .carry(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n302), .c(new_n287), .d(new_n280), .o1(new_n304));
  xorc02aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .out0(new_n305));
  norb02aa1n02x5               g210(.a(new_n303), .b(new_n305), .out0(new_n306));
  aoi022aa1n03x5               g211(.a(new_n304), .b(new_n305), .c(new_n301), .d(new_n306), .o1(\s[29] ));
  xorb03aa1n02x5               g212(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g213(.a(new_n297), .b(new_n305), .c(new_n283), .o(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n293), .c(new_n194), .d(new_n279), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n309), .o1(new_n311));
  tech160nm_fioaoi03aa1n02p5x5 g216(.a(\a[29] ), .b(\b[28] ), .c(new_n303), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n312), .o1(new_n313));
  aoai13aa1n03x5               g218(.a(new_n313), .b(new_n311), .c(new_n287), .d(new_n280), .o1(new_n314));
  xorc02aa1n02x5               g219(.a(\a[30] ), .b(\b[29] ), .out0(new_n315));
  norp02aa1n02x5               g220(.a(new_n312), .b(new_n315), .o1(new_n316));
  aoi022aa1n03x5               g221(.a(new_n314), .b(new_n315), .c(new_n310), .d(new_n316), .o1(\s[30] ));
  nano22aa1n12x5               g222(.a(new_n302), .b(new_n305), .c(new_n315), .out0(new_n318));
  aoai13aa1n02x7               g223(.a(new_n318), .b(new_n293), .c(new_n194), .d(new_n279), .o1(new_n319));
  xorc02aa1n02x5               g224(.a(\a[31] ), .b(\b[30] ), .out0(new_n320));
  inv000aa1d42x5               g225(.a(\a[30] ), .o1(new_n321));
  inv000aa1d42x5               g226(.a(\b[29] ), .o1(new_n322));
  oabi12aa1n02x5               g227(.a(new_n320), .b(\a[30] ), .c(\b[29] ), .out0(new_n323));
  oaoi13aa1n03x5               g228(.a(new_n323), .b(new_n312), .c(new_n321), .d(new_n322), .o1(new_n324));
  inv000aa1d42x5               g229(.a(new_n318), .o1(new_n325));
  oaoi03aa1n02x5               g230(.a(new_n321), .b(new_n322), .c(new_n312), .o1(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n325), .c(new_n287), .d(new_n280), .o1(new_n327));
  aoi022aa1n03x5               g232(.a(new_n327), .b(new_n320), .c(new_n319), .d(new_n324), .o1(\s[31] ));
  inv000aa1d42x5               g233(.a(\a[3] ), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n104), .b(\b[2] ), .c(new_n329), .out0(\s[3] ));
  norp02aa1n02x5               g235(.a(\b[2] ), .b(\a[3] ), .o1(new_n331));
  norp02aa1n03x5               g236(.a(new_n100), .b(new_n104), .o1(new_n332));
  xorc02aa1n02x5               g237(.a(\a[4] ), .b(\b[3] ), .out0(new_n333));
  norp03aa1n02x5               g238(.a(new_n332), .b(new_n333), .c(new_n331), .o1(new_n334));
  aboi22aa1n03x5               g239(.a(new_n332), .b(new_n105), .c(\a[4] ), .d(\b[3] ), .out0(new_n335));
  oaoi13aa1n02x5               g240(.a(new_n334), .b(new_n335), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xorb03aa1n02x5               g241(.a(new_n335), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  xorc02aa1n02x5               g242(.a(\a[6] ), .b(\b[5] ), .out0(new_n338));
  oaoi13aa1n02x5               g243(.a(new_n338), .b(new_n114), .c(new_n335), .d(new_n116), .o1(new_n339));
  oai112aa1n04x5               g244(.a(new_n114), .b(new_n338), .c(new_n335), .d(new_n116), .o1(new_n340));
  norb02aa1n02x5               g245(.a(new_n340), .b(new_n339), .out0(\s[6] ));
  xorc02aa1n02x5               g246(.a(\a[7] ), .b(\b[6] ), .out0(new_n342));
  xnbna2aa1n03x5               g247(.a(new_n342), .b(new_n340), .c(new_n109), .out0(\s[7] ));
  inv000aa1d42x5               g248(.a(new_n112), .o1(new_n344));
  nanp02aa1n02x5               g249(.a(new_n121), .b(new_n122), .o1(new_n345));
  nanp02aa1n02x5               g250(.a(new_n340), .b(new_n109), .o1(new_n346));
  aoai13aa1n02x5               g251(.a(new_n345), .b(new_n344), .c(new_n346), .d(new_n113), .o1(new_n347));
  aoi112aa1n03x4               g252(.a(new_n345), .b(new_n344), .c(new_n346), .d(new_n113), .o1(new_n348));
  nanb02aa1n02x5               g253(.a(new_n348), .b(new_n347), .out0(\s[8] ));
  xnbna2aa1n03x5               g254(.a(new_n129), .b(new_n162), .c(new_n160), .out0(\s[9] ));
endmodule


