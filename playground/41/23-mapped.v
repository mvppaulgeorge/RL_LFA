// Benchmark "adder" written by ABC on Thu Jul 18 09:09:53 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n148, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n177, new_n178, new_n179, new_n180, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n197, new_n199,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n209, new_n210, new_n211, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n228, new_n229, new_n230,
    new_n231, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n243, new_n244, new_n245, new_n246,
    new_n247, new_n249, new_n250, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n257, new_n258, new_n259, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n269, new_n270,
    new_n271, new_n272, new_n273, new_n274, new_n275, new_n277, new_n279,
    new_n280, new_n281, new_n282, new_n283, new_n284, new_n285, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n295,
    new_n297, new_n300, new_n301, new_n303, new_n305;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  nor042aa1n04x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv040aa1d30x5               g003(.a(\a[7] ), .o1(new_n99));
  inv040aa1d32x5               g004(.a(\a[8] ), .o1(new_n100));
  xroi22aa1d06x4               g005(.a(new_n99), .b(\b[6] ), .c(new_n100), .d(\b[7] ), .out0(new_n101));
  nor042aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  aoi022aa1d24x5               g007(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n103));
  nor042aa1n03x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanp02aa1n09x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norb02aa1n06x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nor002aa1n04x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nand42aa1n10x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  norb02aa1n06x4               g013(.a(new_n108), .b(new_n107), .out0(new_n109));
  oai112aa1n06x5               g014(.a(new_n106), .b(new_n109), .c(new_n103), .d(new_n102), .o1(new_n110));
  tech160nm_fiaoi012aa1n04x5   g015(.a(new_n104), .b(new_n107), .c(new_n105), .o1(new_n111));
  nand42aa1n06x5               g016(.a(new_n110), .b(new_n111), .o1(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .out0(new_n113));
  tech160nm_fixnrc02aa1n02p5x5 g018(.a(\b[4] ), .b(\a[5] ), .out0(new_n114));
  nor042aa1n04x5               g019(.a(new_n114), .b(new_n113), .o1(new_n115));
  nanp03aa1d12x5               g020(.a(new_n112), .b(new_n101), .c(new_n115), .o1(new_n116));
  oai022aa1n02x5               g021(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n117));
  aob012aa1n02x5               g022(.a(new_n117), .b(\b[5] ), .c(\a[6] ), .out0(new_n118));
  inv000aa1d42x5               g023(.a(\b[6] ), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(new_n119), .b(new_n99), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[8] ), .b(\b[7] ), .c(new_n120), .o1(new_n121));
  aoib12aa1n06x5               g026(.a(new_n121), .b(new_n101), .c(new_n118), .out0(new_n122));
  nanp02aa1n02x5               g027(.a(new_n116), .b(new_n122), .o1(new_n123));
  xorc02aa1n12x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  nanp02aa1n02x5               g029(.a(new_n123), .b(new_n124), .o1(new_n125));
  nona22aa1n02x4               g030(.a(new_n125), .b(new_n98), .c(new_n97), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n97), .b(new_n98), .c(new_n123), .d(new_n124), .o1(new_n127));
  nanp02aa1n02x5               g032(.a(new_n126), .b(new_n127), .o1(\s[10] ));
  nand42aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  xnrc02aa1n12x5               g034(.a(\b[10] ), .b(\a[11] ), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n126), .c(new_n129), .out0(\s[11] ));
  orn002aa1n12x5               g036(.a(\a[11] ), .b(\b[10] ), .o(new_n132));
  xorc02aa1n12x5               g037(.a(\a[11] ), .b(\b[10] ), .out0(new_n133));
  nanp03aa1n02x5               g038(.a(new_n126), .b(new_n129), .c(new_n133), .o1(new_n134));
  xnrc02aa1n12x5               g039(.a(\b[11] ), .b(\a[12] ), .out0(new_n135));
  nanp03aa1n02x5               g040(.a(new_n134), .b(new_n132), .c(new_n135), .o1(new_n136));
  aoi012aa1n02x5               g041(.a(new_n135), .b(new_n134), .c(new_n132), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(\s[12] ));
  nano23aa1d12x5               g043(.a(new_n135), .b(new_n97), .c(new_n124), .d(new_n133), .out0(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  tech160nm_finor002aa1n05x5   g045(.a(\b[9] ), .b(\a[10] ), .o1(new_n141));
  oai012aa1n12x5               g046(.a(new_n129), .b(new_n98), .c(new_n141), .o1(new_n142));
  oao003aa1n03x5               g047(.a(\a[12] ), .b(\b[11] ), .c(new_n132), .carry(new_n143));
  oai013aa1d12x5               g048(.a(new_n143), .b(new_n135), .c(new_n130), .d(new_n142), .o1(new_n144));
  inv000aa1d42x5               g049(.a(new_n144), .o1(new_n145));
  aoai13aa1n06x5               g050(.a(new_n145), .b(new_n140), .c(new_n116), .d(new_n122), .o1(new_n146));
  xorb03aa1n02x5               g051(.a(new_n146), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g052(.a(\a[14] ), .o1(new_n148));
  nor042aa1d18x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  xorc02aa1n12x5               g054(.a(\a[13] ), .b(\b[12] ), .out0(new_n150));
  aoi012aa1n02x5               g055(.a(new_n149), .b(new_n146), .c(new_n150), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(new_n148), .out0(\s[14] ));
  xorc02aa1n02x5               g057(.a(\a[15] ), .b(\b[14] ), .out0(new_n153));
  xorc02aa1n12x5               g058(.a(\a[14] ), .b(\b[13] ), .out0(new_n154));
  nand42aa1n08x5               g059(.a(new_n154), .b(new_n150), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  inv020aa1d32x5               g061(.a(\b[13] ), .o1(new_n157));
  oaoi03aa1n12x5               g062(.a(new_n148), .b(new_n157), .c(new_n149), .o1(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  aoai13aa1n02x5               g064(.a(new_n153), .b(new_n159), .c(new_n146), .d(new_n156), .o1(new_n160));
  aoi112aa1n02x5               g065(.a(new_n153), .b(new_n159), .c(new_n146), .d(new_n156), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n160), .b(new_n161), .out0(\s[15] ));
  xnrc02aa1n02x5               g067(.a(\b[15] ), .b(\a[16] ), .out0(new_n163));
  oai112aa1n02x5               g068(.a(new_n160), .b(new_n163), .c(\b[14] ), .d(\a[15] ), .o1(new_n164));
  oaoi13aa1n02x5               g069(.a(new_n163), .b(new_n160), .c(\a[15] ), .d(\b[14] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(\s[16] ));
  xorc02aa1n02x5               g071(.a(\a[16] ), .b(\b[15] ), .out0(new_n167));
  nano22aa1n03x7               g072(.a(new_n155), .b(new_n153), .c(new_n167), .out0(new_n168));
  nand02aa1d04x5               g073(.a(new_n139), .b(new_n168), .o1(new_n169));
  xnrc02aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .out0(new_n170));
  aoi112aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n171));
  oab012aa1n02x4               g076(.a(new_n171), .b(\a[16] ), .c(\b[15] ), .out0(new_n172));
  oai013aa1n02x4               g077(.a(new_n172), .b(new_n158), .c(new_n170), .d(new_n163), .o1(new_n173));
  aoi012aa1n06x5               g078(.a(new_n173), .b(new_n168), .c(new_n144), .o1(new_n174));
  aoai13aa1n12x5               g079(.a(new_n174), .b(new_n169), .c(new_n116), .d(new_n122), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g081(.a(\a[18] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(\a[17] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\b[16] ), .o1(new_n179));
  oaoi03aa1n02x5               g084(.a(new_n178), .b(new_n179), .c(new_n175), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[17] ), .c(new_n177), .out0(\s[18] ));
  xroi22aa1d06x4               g086(.a(new_n178), .b(\b[16] ), .c(new_n177), .d(\b[17] ), .out0(new_n182));
  nanp02aa1n02x5               g087(.a(new_n179), .b(new_n178), .o1(new_n183));
  oaoi03aa1n02x5               g088(.a(\a[18] ), .b(\b[17] ), .c(new_n183), .o1(new_n184));
  nor042aa1n06x5               g089(.a(\b[18] ), .b(\a[19] ), .o1(new_n185));
  nanp02aa1n04x5               g090(.a(\b[18] ), .b(\a[19] ), .o1(new_n186));
  norb02aa1n02x5               g091(.a(new_n186), .b(new_n185), .out0(new_n187));
  aoai13aa1n06x5               g092(.a(new_n187), .b(new_n184), .c(new_n175), .d(new_n182), .o1(new_n188));
  aoi112aa1n02x5               g093(.a(new_n187), .b(new_n184), .c(new_n175), .d(new_n182), .o1(new_n189));
  norb02aa1n02x5               g094(.a(new_n188), .b(new_n189), .out0(\s[19] ));
  xnrc02aa1n02x5               g095(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n02x5               g096(.a(\b[19] ), .b(\a[20] ), .o1(new_n192));
  nanp02aa1n04x5               g097(.a(\b[19] ), .b(\a[20] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  nona22aa1n03x5               g099(.a(new_n188), .b(new_n194), .c(new_n185), .out0(new_n195));
  inv000aa1d42x5               g100(.a(new_n185), .o1(new_n196));
  aobi12aa1n02x7               g101(.a(new_n194), .b(new_n188), .c(new_n196), .out0(new_n197));
  norb02aa1n03x4               g102(.a(new_n195), .b(new_n197), .out0(\s[20] ));
  nona23aa1n09x5               g103(.a(new_n193), .b(new_n186), .c(new_n185), .d(new_n192), .out0(new_n199));
  norb02aa1n02x5               g104(.a(new_n182), .b(new_n199), .out0(new_n200));
  oai022aa1n02x5               g105(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n201));
  oaib12aa1n02x5               g106(.a(new_n201), .b(new_n177), .c(\b[17] ), .out0(new_n202));
  tech160nm_fiaoi012aa1n04x5   g107(.a(new_n192), .b(new_n185), .c(new_n193), .o1(new_n203));
  tech160nm_fioai012aa1n05x5   g108(.a(new_n203), .b(new_n199), .c(new_n202), .o1(new_n204));
  xorc02aa1n02x5               g109(.a(\a[21] ), .b(\b[20] ), .out0(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n204), .c(new_n175), .d(new_n200), .o1(new_n206));
  aoi112aa1n02x5               g111(.a(new_n205), .b(new_n204), .c(new_n175), .d(new_n200), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n206), .b(new_n207), .out0(\s[21] ));
  tech160nm_fixnrc02aa1n04x5   g113(.a(\b[21] ), .b(\a[22] ), .out0(new_n209));
  oai112aa1n03x5               g114(.a(new_n206), .b(new_n209), .c(\b[20] ), .d(\a[21] ), .o1(new_n210));
  oaoi13aa1n03x5               g115(.a(new_n209), .b(new_n206), .c(\a[21] ), .d(\b[20] ), .o1(new_n211));
  norb02aa1n02x7               g116(.a(new_n210), .b(new_n211), .out0(\s[22] ));
  nano23aa1n06x5               g117(.a(new_n185), .b(new_n192), .c(new_n193), .d(new_n186), .out0(new_n213));
  xnrc02aa1n02x5               g118(.a(\b[20] ), .b(\a[21] ), .out0(new_n214));
  nor042aa1n02x5               g119(.a(new_n209), .b(new_n214), .o1(new_n215));
  and003aa1n06x5               g120(.a(new_n182), .b(new_n215), .c(new_n213), .o(new_n216));
  inv040aa1n03x5               g121(.a(new_n203), .o1(new_n217));
  aoai13aa1n06x5               g122(.a(new_n215), .b(new_n217), .c(new_n213), .d(new_n184), .o1(new_n218));
  inv000aa1d42x5               g123(.a(\a[22] ), .o1(new_n219));
  inv000aa1d42x5               g124(.a(\b[21] ), .o1(new_n220));
  norp02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  oaoi03aa1n02x5               g126(.a(new_n219), .b(new_n220), .c(new_n221), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(new_n218), .b(new_n222), .o1(new_n223));
  tech160nm_fixorc02aa1n03p5x5 g128(.a(\a[23] ), .b(\b[22] ), .out0(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n223), .c(new_n175), .d(new_n216), .o1(new_n225));
  aoi112aa1n02x5               g130(.a(new_n224), .b(new_n223), .c(new_n175), .d(new_n216), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n225), .b(new_n226), .out0(\s[23] ));
  xorc02aa1n12x5               g132(.a(\a[24] ), .b(\b[23] ), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  oai112aa1n03x5               g134(.a(new_n225), .b(new_n229), .c(\b[22] ), .d(\a[23] ), .o1(new_n230));
  oaoi13aa1n03x5               g135(.a(new_n229), .b(new_n225), .c(\a[23] ), .d(\b[22] ), .o1(new_n231));
  norb02aa1n02x7               g136(.a(new_n230), .b(new_n231), .out0(\s[24] ));
  and002aa1n06x5               g137(.a(new_n228), .b(new_n224), .o(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  nano32aa1n02x4               g139(.a(new_n234), .b(new_n182), .c(new_n213), .d(new_n215), .out0(new_n235));
  aoi112aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n236));
  oab012aa1n02x4               g141(.a(new_n236), .b(\a[24] ), .c(\b[23] ), .out0(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n234), .c(new_n218), .d(new_n222), .o1(new_n238));
  tech160nm_fixorc02aa1n02p5x5 g143(.a(\a[25] ), .b(\b[24] ), .out0(new_n239));
  aoai13aa1n06x5               g144(.a(new_n239), .b(new_n238), .c(new_n175), .d(new_n235), .o1(new_n240));
  aoi112aa1n02x5               g145(.a(new_n239), .b(new_n238), .c(new_n175), .d(new_n235), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n240), .b(new_n241), .out0(\s[25] ));
  norp02aa1n02x5               g147(.a(\b[24] ), .b(\a[25] ), .o1(new_n243));
  xorc02aa1n12x5               g148(.a(\a[26] ), .b(\b[25] ), .out0(new_n244));
  nona22aa1n03x5               g149(.a(new_n240), .b(new_n244), .c(new_n243), .out0(new_n245));
  inv000aa1d42x5               g150(.a(new_n244), .o1(new_n246));
  oaoi13aa1n06x5               g151(.a(new_n246), .b(new_n240), .c(\a[25] ), .d(\b[24] ), .o1(new_n247));
  norb02aa1n03x4               g152(.a(new_n245), .b(new_n247), .out0(\s[26] ));
  and002aa1n06x5               g153(.a(new_n244), .b(new_n239), .o(new_n249));
  and003aa1n06x5               g154(.a(new_n216), .b(new_n249), .c(new_n233), .o(new_n250));
  nand02aa1d06x5               g155(.a(new_n175), .b(new_n250), .o1(new_n251));
  orn002aa1n02x5               g156(.a(\a[25] ), .b(\b[24] ), .o(new_n252));
  oao003aa1n02x5               g157(.a(\a[26] ), .b(\b[25] ), .c(new_n252), .carry(new_n253));
  aobi12aa1n06x5               g158(.a(new_n253), .b(new_n238), .c(new_n249), .out0(new_n254));
  xorc02aa1n02x5               g159(.a(\a[27] ), .b(\b[26] ), .out0(new_n255));
  xnbna2aa1n03x5               g160(.a(new_n255), .b(new_n251), .c(new_n254), .out0(\s[27] ));
  norp02aa1n02x5               g161(.a(\b[26] ), .b(\a[27] ), .o1(new_n257));
  inv040aa1n03x5               g162(.a(new_n257), .o1(new_n258));
  aobi12aa1n03x5               g163(.a(new_n255), .b(new_n251), .c(new_n254), .out0(new_n259));
  xnrc02aa1n02x5               g164(.a(\b[27] ), .b(\a[28] ), .out0(new_n260));
  nano22aa1n03x5               g165(.a(new_n259), .b(new_n258), .c(new_n260), .out0(new_n261));
  inv000aa1n02x5               g166(.a(new_n222), .o1(new_n262));
  aoai13aa1n02x5               g167(.a(new_n233), .b(new_n262), .c(new_n204), .d(new_n215), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n249), .o1(new_n264));
  aoai13aa1n06x5               g169(.a(new_n253), .b(new_n264), .c(new_n263), .d(new_n237), .o1(new_n265));
  aoai13aa1n02x5               g170(.a(new_n255), .b(new_n265), .c(new_n175), .d(new_n250), .o1(new_n266));
  aoi012aa1n03x5               g171(.a(new_n260), .b(new_n266), .c(new_n258), .o1(new_n267));
  norp02aa1n03x5               g172(.a(new_n267), .b(new_n261), .o1(\s[28] ));
  norb02aa1n02x5               g173(.a(new_n255), .b(new_n260), .out0(new_n269));
  aobi12aa1n06x5               g174(.a(new_n269), .b(new_n251), .c(new_n254), .out0(new_n270));
  oao003aa1n02x5               g175(.a(\a[28] ), .b(\b[27] ), .c(new_n258), .carry(new_n271));
  xnrc02aa1n02x5               g176(.a(\b[28] ), .b(\a[29] ), .out0(new_n272));
  nano22aa1n03x7               g177(.a(new_n270), .b(new_n271), .c(new_n272), .out0(new_n273));
  aoai13aa1n02x5               g178(.a(new_n269), .b(new_n265), .c(new_n175), .d(new_n250), .o1(new_n274));
  aoi012aa1n02x5               g179(.a(new_n272), .b(new_n274), .c(new_n271), .o1(new_n275));
  norp02aa1n03x5               g180(.a(new_n275), .b(new_n273), .o1(\s[29] ));
  nanp02aa1n02x5               g181(.a(\b[0] ), .b(\a[1] ), .o1(new_n277));
  xorb03aa1n02x5               g182(.a(new_n277), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g183(.a(new_n255), .b(new_n272), .c(new_n260), .out0(new_n279));
  aobi12aa1n06x5               g184(.a(new_n279), .b(new_n251), .c(new_n254), .out0(new_n280));
  oao003aa1n02x5               g185(.a(\a[29] ), .b(\b[28] ), .c(new_n271), .carry(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[29] ), .b(\a[30] ), .out0(new_n282));
  nano22aa1n03x7               g187(.a(new_n280), .b(new_n281), .c(new_n282), .out0(new_n283));
  aoai13aa1n02x5               g188(.a(new_n279), .b(new_n265), .c(new_n175), .d(new_n250), .o1(new_n284));
  aoi012aa1n02x5               g189(.a(new_n282), .b(new_n284), .c(new_n281), .o1(new_n285));
  norp02aa1n03x5               g190(.a(new_n285), .b(new_n283), .o1(\s[30] ));
  norb02aa1n02x5               g191(.a(new_n279), .b(new_n282), .out0(new_n287));
  aobi12aa1n06x5               g192(.a(new_n287), .b(new_n251), .c(new_n254), .out0(new_n288));
  oao003aa1n02x5               g193(.a(\a[30] ), .b(\b[29] ), .c(new_n281), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[30] ), .b(\a[31] ), .out0(new_n290));
  nano22aa1n03x7               g195(.a(new_n288), .b(new_n289), .c(new_n290), .out0(new_n291));
  aoai13aa1n02x5               g196(.a(new_n287), .b(new_n265), .c(new_n175), .d(new_n250), .o1(new_n292));
  aoi012aa1n02x5               g197(.a(new_n290), .b(new_n292), .c(new_n289), .o1(new_n293));
  norp02aa1n03x5               g198(.a(new_n293), .b(new_n291), .o1(\s[31] ));
  oabi12aa1n02x5               g199(.a(new_n103), .b(\a[2] ), .c(\b[1] ), .out0(new_n295));
  xorb03aa1n02x5               g200(.a(new_n295), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi13aa1n02x5               g201(.a(new_n107), .b(new_n108), .c(new_n103), .d(new_n102), .o1(new_n297));
  xnrc02aa1n02x5               g202(.a(new_n297), .b(new_n106), .out0(\s[4] ));
  xobna2aa1n03x5               g203(.a(new_n114), .b(new_n110), .c(new_n111), .out0(\s[5] ));
  orn002aa1n02x5               g204(.a(\a[5] ), .b(\b[4] ), .o(new_n300));
  aoai13aa1n02x5               g205(.a(new_n300), .b(new_n114), .c(new_n110), .d(new_n111), .o1(new_n301));
  xorb03aa1n02x5               g206(.a(new_n301), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aob012aa1n02x5               g207(.a(new_n118), .b(new_n112), .c(new_n115), .out0(new_n303));
  xorb03aa1n02x5               g208(.a(new_n303), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g209(.a(new_n99), .b(new_n119), .c(new_n303), .o1(new_n305));
  xorb03aa1n02x5               g210(.a(new_n305), .b(\b[7] ), .c(new_n100), .out0(\s[8] ));
  xnbna2aa1n03x5               g211(.a(new_n124), .b(new_n116), .c(new_n122), .out0(\s[9] ));
endmodule

