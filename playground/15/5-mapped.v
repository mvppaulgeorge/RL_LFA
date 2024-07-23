// Benchmark "adder" written by ABC on Wed Jul 17 19:39:19 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n301, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n311, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n322, new_n323, new_n326,
    new_n328, new_n329, new_n330;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n12x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor002aa1n04x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nand22aa1n04x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nor002aa1d32x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  tech160nm_finand02aa1n03p5x5 g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nona23aa1n09x5               g007(.a(new_n102), .b(new_n100), .c(new_n99), .d(new_n101), .out0(new_n103));
  xorc02aa1n02x5               g008(.a(\a[5] ), .b(\b[4] ), .out0(new_n104));
  tech160nm_fixorc02aa1n02p5x5 g009(.a(\a[6] ), .b(\b[5] ), .out0(new_n105));
  nano22aa1n03x7               g010(.a(new_n103), .b(new_n104), .c(new_n105), .out0(new_n106));
  and002aa1n12x5               g011(.a(\b[3] ), .b(\a[4] ), .o(new_n107));
  nor042aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nor022aa1n12x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nor003aa1n03x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  nand42aa1n04x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nanb02aa1n12x5               g016(.a(new_n109), .b(new_n111), .out0(new_n112));
  oai112aa1n06x5               g017(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n113));
  nanp02aa1n03x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  nand02aa1d04x5               g019(.a(new_n113), .b(new_n114), .o1(new_n115));
  oaoi13aa1n12x5               g020(.a(new_n107), .b(new_n110), .c(new_n115), .d(new_n112), .o1(new_n116));
  nor002aa1d32x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nor042aa1n03x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  and002aa1n06x5               g023(.a(\b[5] ), .b(\a[6] ), .o(new_n119));
  oab012aa1n02x5               g024(.a(new_n119), .b(new_n117), .c(new_n118), .out0(new_n120));
  oai012aa1n02x5               g025(.a(new_n100), .b(new_n101), .c(new_n99), .o1(new_n121));
  oaib12aa1n06x5               g026(.a(new_n121), .b(new_n103), .c(new_n120), .out0(new_n122));
  tech160nm_fixorc02aa1n05x5   g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n116), .d(new_n106), .o1(new_n124));
  nor042aa1n02x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand42aa1n06x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb02aa1n03x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n124), .c(new_n98), .out0(\s[10] ));
  inv030aa1n02x5               g033(.a(new_n117), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[4] ), .b(\a[5] ), .o1(new_n130));
  nano23aa1n02x4               g035(.a(new_n119), .b(new_n118), .c(new_n129), .d(new_n130), .out0(new_n131));
  oai012aa1n06x5               g036(.a(new_n110), .b(new_n115), .c(new_n112), .o1(new_n132));
  nona23aa1n06x5               g037(.a(new_n132), .b(new_n131), .c(new_n103), .d(new_n107), .out0(new_n133));
  norb02aa1n03x5               g038(.a(new_n100), .b(new_n99), .out0(new_n134));
  inv000aa1d42x5               g039(.a(new_n101), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(new_n134), .b(new_n135), .o1(new_n136));
  aboi22aa1n03x5               g041(.a(new_n103), .b(new_n120), .c(new_n136), .d(new_n100), .out0(new_n137));
  nanp02aa1n02x5               g042(.a(new_n123), .b(new_n127), .o1(new_n138));
  tech160nm_fioai012aa1n04x5   g043(.a(new_n126), .b(new_n125), .c(new_n97), .o1(new_n139));
  aoai13aa1n02x5               g044(.a(new_n139), .b(new_n138), .c(new_n133), .d(new_n137), .o1(new_n140));
  xorb03aa1n02x5               g045(.a(new_n140), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  orn002aa1n02x5               g046(.a(\a[11] ), .b(\b[10] ), .o(new_n142));
  nor022aa1n08x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  nanp02aa1n06x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  nanp02aa1n03x5               g050(.a(new_n140), .b(new_n145), .o1(new_n146));
  nor002aa1n06x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nand42aa1n04x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n149), .b(new_n146), .c(new_n142), .out0(\s[12] ));
  nona23aa1n09x5               g055(.a(new_n148), .b(new_n144), .c(new_n143), .d(new_n147), .out0(new_n151));
  nano22aa1n02x4               g056(.a(new_n151), .b(new_n123), .c(new_n127), .out0(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n122), .c(new_n116), .d(new_n106), .o1(new_n153));
  oai012aa1n02x5               g058(.a(new_n148), .b(new_n147), .c(new_n143), .o1(new_n154));
  tech160nm_fioai012aa1n04x5   g059(.a(new_n154), .b(new_n151), .c(new_n139), .o1(new_n155));
  nanb02aa1n02x5               g060(.a(new_n155), .b(new_n153), .out0(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  nand42aa1n08x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nano22aa1n02x4               g064(.a(new_n158), .b(new_n156), .c(new_n159), .out0(new_n160));
  nor002aa1n06x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand42aa1n08x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanb02aa1n02x5               g067(.a(new_n161), .b(new_n162), .out0(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n158), .c(new_n156), .d(new_n159), .o1(new_n164));
  nona22aa1n02x4               g069(.a(new_n162), .b(new_n161), .c(new_n158), .out0(new_n165));
  oaih12aa1n02x5               g070(.a(new_n164), .b(new_n160), .c(new_n165), .o1(\s[14] ));
  nona23aa1n09x5               g071(.a(new_n162), .b(new_n159), .c(new_n158), .d(new_n161), .out0(new_n167));
  nano23aa1n06x5               g072(.a(new_n158), .b(new_n161), .c(new_n162), .d(new_n159), .out0(new_n168));
  aoi022aa1n02x5               g073(.a(new_n155), .b(new_n168), .c(new_n162), .d(new_n165), .o1(new_n169));
  oai012aa1n02x5               g074(.a(new_n169), .b(new_n153), .c(new_n167), .o1(new_n170));
  xorb03aa1n02x5               g075(.a(new_n170), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nand42aa1n03x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nanb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(new_n174));
  oaoi13aa1n02x5               g079(.a(new_n174), .b(new_n169), .c(new_n153), .d(new_n167), .o1(new_n175));
  nor042aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  tech160nm_finand02aa1n03p5x5 g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nanb02aa1n02x5               g082(.a(new_n176), .b(new_n177), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n172), .c(new_n170), .d(new_n173), .o1(new_n179));
  nona22aa1n03x5               g084(.a(new_n177), .b(new_n176), .c(new_n172), .out0(new_n180));
  oai012aa1n02x5               g085(.a(new_n179), .b(new_n175), .c(new_n180), .o1(\s[16] ));
  nano23aa1n06x5               g086(.a(new_n143), .b(new_n147), .c(new_n148), .d(new_n144), .out0(new_n182));
  nano23aa1n03x7               g087(.a(new_n172), .b(new_n176), .c(new_n177), .d(new_n173), .out0(new_n183));
  nano32aa1n03x7               g088(.a(new_n138), .b(new_n183), .c(new_n182), .d(new_n168), .out0(new_n184));
  aoai13aa1n12x5               g089(.a(new_n184), .b(new_n122), .c(new_n116), .d(new_n106), .o1(new_n185));
  nor043aa1n02x5               g090(.a(new_n167), .b(new_n174), .c(new_n178), .o1(new_n186));
  oa0012aa1n02x5               g091(.a(new_n162), .b(new_n161), .c(new_n158), .o(new_n187));
  aoi022aa1n02x7               g092(.a(new_n183), .b(new_n187), .c(new_n177), .d(new_n180), .o1(new_n188));
  aobi12aa1n12x5               g093(.a(new_n188), .b(new_n186), .c(new_n155), .out0(new_n189));
  tech160nm_fixnrc02aa1n04x5   g094(.a(\b[16] ), .b(\a[17] ), .out0(new_n190));
  xobna2aa1n03x5               g095(.a(new_n190), .b(new_n185), .c(new_n189), .out0(\s[17] ));
  nor002aa1d32x5               g096(.a(\b[16] ), .b(\a[17] ), .o1(new_n192));
  inv040aa1n12x5               g097(.a(new_n192), .o1(new_n193));
  aoai13aa1n02x5               g098(.a(new_n193), .b(new_n190), .c(new_n185), .d(new_n189), .o1(new_n194));
  tech160nm_fixnrc02aa1n05x5   g099(.a(\b[17] ), .b(\a[18] ), .out0(new_n195));
  norp02aa1n02x5               g100(.a(new_n195), .b(new_n192), .o1(new_n196));
  aoai13aa1n02x5               g101(.a(new_n196), .b(new_n190), .c(new_n185), .d(new_n189), .o1(new_n197));
  aob012aa1n02x5               g102(.a(new_n197), .b(new_n194), .c(new_n195), .out0(\s[18] ));
  nanp02aa1n02x5               g103(.a(new_n133), .b(new_n137), .o1(new_n199));
  aob012aa1n03x5               g104(.a(new_n188), .b(new_n155), .c(new_n186), .out0(new_n200));
  nor042aa1n02x5               g105(.a(new_n195), .b(new_n190), .o1(new_n201));
  aoai13aa1n03x5               g106(.a(new_n201), .b(new_n200), .c(new_n199), .d(new_n184), .o1(new_n202));
  nor042aa1n04x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nand42aa1n02x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nanb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  oaoi03aa1n12x5               g111(.a(\a[18] ), .b(\b[17] ), .c(new_n193), .o1(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  xnbna2aa1n03x5               g113(.a(new_n206), .b(new_n202), .c(new_n208), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g115(.a(new_n201), .o1(new_n211));
  aoai13aa1n02x7               g116(.a(new_n208), .b(new_n211), .c(new_n185), .d(new_n189), .o1(new_n212));
  nor042aa1n03x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nanp02aa1n04x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nanb02aa1n02x5               g119(.a(new_n213), .b(new_n214), .out0(new_n215));
  aoai13aa1n03x5               g120(.a(new_n215), .b(new_n203), .c(new_n212), .d(new_n206), .o1(new_n216));
  norb03aa1n02x5               g121(.a(new_n214), .b(new_n203), .c(new_n213), .out0(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n205), .c(new_n202), .d(new_n208), .o1(new_n218));
  nanp02aa1n02x5               g123(.a(new_n216), .b(new_n218), .o1(\s[20] ));
  nano23aa1n09x5               g124(.a(new_n203), .b(new_n213), .c(new_n214), .d(new_n204), .out0(new_n220));
  nona22aa1n03x5               g125(.a(new_n220), .b(new_n195), .c(new_n190), .out0(new_n221));
  tech160nm_fioai012aa1n03p5x5 g126(.a(new_n214), .b(new_n213), .c(new_n203), .o1(new_n222));
  aobi12aa1n09x5               g127(.a(new_n222), .b(new_n220), .c(new_n207), .out0(new_n223));
  aoai13aa1n06x5               g128(.a(new_n223), .b(new_n221), .c(new_n185), .d(new_n189), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  nand42aa1n08x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n227), .b(new_n226), .out0(new_n228));
  nor042aa1d18x5               g133(.a(\b[21] ), .b(\a[22] ), .o1(new_n229));
  nanp02aa1n12x5               g134(.a(\b[21] ), .b(\a[22] ), .o1(new_n230));
  norb02aa1n02x5               g135(.a(new_n230), .b(new_n229), .out0(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n03x5               g137(.a(new_n232), .b(new_n226), .c(new_n224), .d(new_n228), .o1(new_n233));
  nand42aa1n02x5               g138(.a(new_n224), .b(new_n228), .o1(new_n234));
  norb03aa1n02x5               g139(.a(new_n230), .b(new_n226), .c(new_n229), .out0(new_n235));
  nand02aa1n02x5               g140(.a(new_n234), .b(new_n235), .o1(new_n236));
  nanp02aa1n03x5               g141(.a(new_n233), .b(new_n236), .o1(\s[22] ));
  nano23aa1n03x7               g142(.a(new_n226), .b(new_n229), .c(new_n230), .d(new_n227), .out0(new_n238));
  nona23aa1n02x4               g143(.a(new_n220), .b(new_n238), .c(new_n195), .d(new_n190), .out0(new_n239));
  inv000aa1n02x5               g144(.a(new_n223), .o1(new_n240));
  and002aa1n02x5               g145(.a(\b[21] ), .b(\a[22] ), .o(new_n241));
  oab012aa1n02x4               g146(.a(new_n241), .b(new_n226), .c(new_n229), .out0(new_n242));
  tech160nm_fiaoi012aa1n03p5x5 g147(.a(new_n242), .b(new_n240), .c(new_n238), .o1(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n239), .c(new_n185), .d(new_n189), .o1(new_n244));
  xorb03aa1n02x5               g149(.a(new_n244), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1d32x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  nand02aa1n20x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n247), .b(new_n246), .out0(new_n248));
  nor022aa1n16x5               g153(.a(\b[23] ), .b(\a[24] ), .o1(new_n249));
  nand02aa1d10x5               g154(.a(\b[23] ), .b(\a[24] ), .o1(new_n250));
  nanb02aa1n02x5               g155(.a(new_n249), .b(new_n250), .out0(new_n251));
  aoai13aa1n03x5               g156(.a(new_n251), .b(new_n246), .c(new_n244), .d(new_n248), .o1(new_n252));
  nand42aa1n02x5               g157(.a(new_n244), .b(new_n248), .o1(new_n253));
  nona22aa1d24x5               g158(.a(new_n250), .b(new_n249), .c(new_n246), .out0(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  nanp02aa1n03x5               g160(.a(new_n253), .b(new_n255), .o1(new_n256));
  nanp02aa1n03x5               g161(.a(new_n252), .b(new_n256), .o1(\s[24] ));
  nano23aa1n09x5               g162(.a(new_n246), .b(new_n249), .c(new_n250), .d(new_n247), .out0(new_n258));
  nand02aa1n03x5               g163(.a(new_n258), .b(new_n238), .o1(new_n259));
  nanb03aa1n06x5               g164(.a(new_n259), .b(new_n201), .c(new_n220), .out0(new_n260));
  nand02aa1d04x5               g165(.a(new_n220), .b(new_n207), .o1(new_n261));
  aoi022aa1n06x5               g166(.a(new_n258), .b(new_n242), .c(new_n250), .d(new_n254), .o1(new_n262));
  aoai13aa1n06x5               g167(.a(new_n262), .b(new_n259), .c(new_n261), .d(new_n222), .o1(new_n263));
  inv000aa1n02x5               g168(.a(new_n263), .o1(new_n264));
  aoai13aa1n04x5               g169(.a(new_n264), .b(new_n260), .c(new_n185), .d(new_n189), .o1(new_n265));
  xorb03aa1n02x5               g170(.a(new_n265), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g171(.a(\b[24] ), .b(\a[25] ), .o1(new_n267));
  xorc02aa1n06x5               g172(.a(\a[25] ), .b(\b[24] ), .out0(new_n268));
  xnrc02aa1n03x5               g173(.a(\b[25] ), .b(\a[26] ), .out0(new_n269));
  aoai13aa1n03x5               g174(.a(new_n269), .b(new_n267), .c(new_n265), .d(new_n268), .o1(new_n270));
  nanp02aa1n02x5               g175(.a(new_n265), .b(new_n268), .o1(new_n271));
  oabi12aa1n12x5               g176(.a(new_n269), .b(\a[25] ), .c(\b[24] ), .out0(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  nand02aa1n02x5               g178(.a(new_n271), .b(new_n273), .o1(new_n274));
  nanp02aa1n03x5               g179(.a(new_n270), .b(new_n274), .o1(\s[26] ));
  norb02aa1n02x5               g180(.a(new_n268), .b(new_n269), .out0(new_n276));
  norb03aa1n09x5               g181(.a(new_n276), .b(new_n221), .c(new_n259), .out0(new_n277));
  inv000aa1n02x5               g182(.a(new_n277), .o1(new_n278));
  nanp02aa1n02x5               g183(.a(\b[25] ), .b(\a[26] ), .o1(new_n279));
  aoi022aa1n12x5               g184(.a(new_n263), .b(new_n276), .c(new_n279), .d(new_n272), .o1(new_n280));
  aoai13aa1n09x5               g185(.a(new_n280), .b(new_n278), .c(new_n185), .d(new_n189), .o1(new_n281));
  xorb03aa1n03x5               g186(.a(new_n281), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  xorc02aa1n12x5               g188(.a(\a[27] ), .b(\b[26] ), .out0(new_n284));
  xnrc02aa1n12x5               g189(.a(\b[27] ), .b(\a[28] ), .out0(new_n285));
  aoai13aa1n03x5               g190(.a(new_n285), .b(new_n283), .c(new_n281), .d(new_n284), .o1(new_n286));
  aoai13aa1n03x5               g191(.a(new_n277), .b(new_n200), .c(new_n199), .d(new_n184), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n284), .o1(new_n288));
  norp02aa1n02x5               g193(.a(new_n285), .b(new_n283), .o1(new_n289));
  aoai13aa1n02x5               g194(.a(new_n289), .b(new_n288), .c(new_n287), .d(new_n280), .o1(new_n290));
  nanp02aa1n03x5               g195(.a(new_n286), .b(new_n290), .o1(\s[28] ));
  norb02aa1n06x5               g196(.a(new_n284), .b(new_n285), .out0(new_n292));
  inv000aa1d42x5               g197(.a(new_n292), .o1(new_n293));
  xorc02aa1n12x5               g198(.a(\a[29] ), .b(\b[28] ), .out0(new_n294));
  inv000aa1d42x5               g199(.a(new_n294), .o1(new_n295));
  tech160nm_fiaoi012aa1n05x5   g200(.a(new_n289), .b(\a[28] ), .c(\b[27] ), .o1(new_n296));
  norp02aa1n02x5               g201(.a(new_n296), .b(new_n295), .o1(new_n297));
  aoai13aa1n02x5               g202(.a(new_n297), .b(new_n293), .c(new_n287), .d(new_n280), .o1(new_n298));
  aoai13aa1n03x5               g203(.a(new_n295), .b(new_n296), .c(new_n281), .d(new_n292), .o1(new_n299));
  nanp02aa1n03x5               g204(.a(new_n299), .b(new_n298), .o1(\s[29] ));
  nanp02aa1n02x5               g205(.a(\b[0] ), .b(\a[1] ), .o1(new_n301));
  xorb03aa1n02x5               g206(.a(new_n301), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x5               g207(.a(new_n285), .b(new_n284), .c(new_n294), .out0(new_n303));
  nanp02aa1n03x5               g208(.a(new_n281), .b(new_n303), .o1(new_n304));
  norp02aa1n02x5               g209(.a(\b[28] ), .b(\a[29] ), .o1(new_n305));
  aoi012aa1n02x5               g210(.a(new_n305), .b(new_n296), .c(new_n294), .o1(new_n306));
  xorc02aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .out0(new_n307));
  inv000aa1n02x5               g212(.a(new_n303), .o1(new_n308));
  oai012aa1n02x5               g213(.a(new_n307), .b(\b[28] ), .c(\a[29] ), .o1(new_n309));
  aoi012aa1n02x5               g214(.a(new_n309), .b(new_n296), .c(new_n294), .o1(new_n310));
  aoai13aa1n02x7               g215(.a(new_n310), .b(new_n308), .c(new_n287), .d(new_n280), .o1(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n307), .c(new_n304), .d(new_n306), .o1(\s[30] ));
  nano23aa1n06x5               g217(.a(new_n295), .b(new_n285), .c(new_n307), .d(new_n284), .out0(new_n313));
  inv000aa1d42x5               g218(.a(new_n313), .o1(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[30] ), .b(\a[31] ), .out0(new_n315));
  aoi012aa1n02x5               g220(.a(new_n310), .b(\a[30] ), .c(\b[29] ), .o1(new_n316));
  norp02aa1n02x5               g221(.a(new_n316), .b(new_n315), .o1(new_n317));
  aoai13aa1n02x5               g222(.a(new_n317), .b(new_n314), .c(new_n287), .d(new_n280), .o1(new_n318));
  aoai13aa1n03x5               g223(.a(new_n315), .b(new_n316), .c(new_n281), .d(new_n313), .o1(new_n319));
  nanp02aa1n03x5               g224(.a(new_n319), .b(new_n318), .o1(\s[31] ));
  xnbna2aa1n03x5               g225(.a(new_n112), .b(new_n113), .c(new_n114), .out0(\s[3] ));
  norp02aa1n02x5               g226(.a(new_n107), .b(new_n108), .o1(new_n322));
  aoi013aa1n02x4               g227(.a(new_n109), .b(new_n113), .c(new_n114), .d(new_n111), .o1(new_n323));
  oai012aa1n02x5               g228(.a(new_n132), .b(new_n323), .c(new_n322), .o1(\s[4] ));
  xorb03aa1n02x5               g229(.a(new_n116), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb03aa1n02x5               g230(.a(new_n107), .b(new_n132), .c(new_n104), .out0(new_n326));
  xnbna2aa1n03x5               g231(.a(new_n105), .b(new_n326), .c(new_n129), .out0(\s[6] ));
  nona32aa1n02x4               g232(.a(new_n326), .b(new_n119), .c(new_n118), .d(new_n117), .out0(new_n328));
  nona23aa1n03x5               g233(.a(new_n328), .b(new_n102), .c(new_n101), .d(new_n119), .out0(new_n329));
  aboi22aa1n03x5               g234(.a(new_n119), .b(new_n328), .c(new_n135), .d(new_n102), .out0(new_n330));
  norb02aa1n02x5               g235(.a(new_n329), .b(new_n330), .out0(\s[7] ));
  xnbna2aa1n03x5               g236(.a(new_n134), .b(new_n329), .c(new_n135), .out0(\s[8] ));
  xnbna2aa1n03x5               g237(.a(new_n123), .b(new_n133), .c(new_n137), .out0(\s[9] ));
endmodule


