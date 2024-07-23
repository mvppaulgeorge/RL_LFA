// Benchmark "adder" written by ABC on Thu Jul 18 11:38:08 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n321, new_n323, new_n324, new_n326;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n12x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nor042aa1n04x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oai012aa1n04x7               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  norp02aa1n09x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand42aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n16x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  tech160nm_fiaoi012aa1n03p5x5 g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nor022aa1n08x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand02aa1d08x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor022aa1n16x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1d18x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  tech160nm_fixnrc02aa1n02p5x5 g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  tech160nm_fixnrc02aa1n04x5   g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nor043aa1n06x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  inv040aa1d32x5               g022(.a(\a[5] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[4] ), .o1(new_n119));
  nand22aa1n03x5               g024(.a(new_n119), .b(new_n118), .o1(new_n120));
  oao003aa1n06x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .carry(new_n121));
  ao0012aa1n03x7               g026(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n122));
  oabi12aa1n18x5               g027(.a(new_n122), .b(new_n121), .c(new_n114), .out0(new_n123));
  tech160nm_fixorc02aa1n04x5   g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n06x5               g029(.a(new_n124), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n125));
  nor002aa1d32x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand02aa1n08x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n06x4               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  aobi12aa1n06x5               g034(.a(new_n128), .b(new_n125), .c(new_n98), .out0(new_n130));
  nor002aa1d32x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand02aa1d12x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  oai012aa1d24x5               g038(.a(new_n127), .b(new_n126), .c(new_n97), .o1(new_n134));
  inv020aa1n06x5               g039(.a(new_n134), .o1(new_n135));
  oai012aa1n04x7               g040(.a(new_n133), .b(new_n130), .c(new_n135), .o1(new_n136));
  norp03aa1n02x5               g041(.a(new_n130), .b(new_n133), .c(new_n135), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(\s[11] ));
  nor002aa1d32x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanp02aa1n12x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  nona22aa1n02x5               g046(.a(new_n136), .b(new_n141), .c(new_n131), .out0(new_n142));
  inv040aa1n09x5               g047(.a(new_n131), .o1(new_n143));
  aobi12aa1n06x5               g048(.a(new_n141), .b(new_n136), .c(new_n143), .out0(new_n144));
  norb02aa1n02x7               g049(.a(new_n142), .b(new_n144), .out0(\s[12] ));
  nano23aa1n09x5               g050(.a(new_n131), .b(new_n139), .c(new_n140), .d(new_n132), .out0(new_n146));
  nand23aa1n06x5               g051(.a(new_n146), .b(new_n124), .c(new_n128), .o1(new_n147));
  inv030aa1n02x5               g052(.a(new_n147), .o1(new_n148));
  aoai13aa1n06x5               g053(.a(new_n148), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n149));
  nona23aa1n09x5               g054(.a(new_n140), .b(new_n132), .c(new_n131), .d(new_n139), .out0(new_n150));
  oaoi03aa1n12x5               g055(.a(\a[12] ), .b(\b[11] ), .c(new_n143), .o1(new_n151));
  oabi12aa1n18x5               g056(.a(new_n151), .b(new_n150), .c(new_n134), .out0(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nanp02aa1n06x5               g058(.a(new_n149), .b(new_n153), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  tech160nm_finand02aa1n05x5   g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n156), .b(new_n154), .c(new_n157), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n06x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nand42aa1n03x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  nanb02aa1n02x5               g066(.a(new_n160), .b(new_n161), .out0(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  nor042aa1n06x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand02aa1n16x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  aoi012aa1d18x5               g070(.a(new_n164), .b(new_n156), .c(new_n165), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  nano23aa1d15x5               g072(.a(new_n156), .b(new_n164), .c(new_n165), .d(new_n157), .out0(new_n168));
  aoai13aa1n06x5               g073(.a(new_n163), .b(new_n167), .c(new_n154), .d(new_n168), .o1(new_n169));
  aoi112aa1n02x5               g074(.a(new_n167), .b(new_n163), .c(new_n154), .d(new_n168), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n169), .b(new_n170), .out0(\s[15] ));
  inv000aa1d42x5               g076(.a(new_n160), .o1(new_n172));
  nor042aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanp02aa1n04x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nanb02aa1n02x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  nanp03aa1n02x5               g080(.a(new_n169), .b(new_n172), .c(new_n175), .o1(new_n176));
  tech160nm_fiaoi012aa1n05x5   g081(.a(new_n175), .b(new_n169), .c(new_n172), .o1(new_n177));
  norb02aa1n03x4               g082(.a(new_n176), .b(new_n177), .out0(\s[16] ));
  nano23aa1n09x5               g083(.a(new_n160), .b(new_n173), .c(new_n174), .d(new_n161), .out0(new_n179));
  nano22aa1n12x5               g084(.a(new_n147), .b(new_n168), .c(new_n179), .out0(new_n180));
  aoai13aa1n06x5               g085(.a(new_n180), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n181));
  aoai13aa1n06x5               g086(.a(new_n179), .b(new_n167), .c(new_n152), .d(new_n168), .o1(new_n182));
  tech160nm_fiaoi012aa1n04x5   g087(.a(new_n173), .b(new_n160), .c(new_n174), .o1(new_n183));
  nand23aa1n06x5               g088(.a(new_n181), .b(new_n182), .c(new_n183), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g090(.a(\a[18] ), .o1(new_n186));
  inv040aa1d32x5               g091(.a(\a[17] ), .o1(new_n187));
  inv000aa1d48x5               g092(.a(\b[16] ), .o1(new_n188));
  oaoi03aa1n03x5               g093(.a(new_n187), .b(new_n188), .c(new_n184), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[17] ), .c(new_n186), .out0(\s[18] ));
  nand22aa1n04x5               g095(.a(new_n109), .b(new_n117), .o1(new_n191));
  nanb02aa1n12x5               g096(.a(new_n123), .b(new_n191), .out0(new_n192));
  inv030aa1n02x5               g097(.a(new_n179), .o1(new_n193));
  aoai13aa1n06x5               g098(.a(new_n168), .b(new_n151), .c(new_n146), .d(new_n135), .o1(new_n194));
  aoai13aa1n09x5               g099(.a(new_n183), .b(new_n193), .c(new_n194), .d(new_n166), .o1(new_n195));
  xroi22aa1d06x4               g100(.a(new_n187), .b(\b[16] ), .c(new_n186), .d(\b[17] ), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n195), .c(new_n192), .d(new_n180), .o1(new_n197));
  oai022aa1n04x7               g102(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n198));
  oaib12aa1n09x5               g103(.a(new_n198), .b(new_n186), .c(\b[17] ), .out0(new_n199));
  nor002aa1d32x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nand42aa1d28x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  xnbna2aa1n03x5               g108(.a(new_n203), .b(new_n197), .c(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g110(.a(new_n200), .o1(new_n206));
  aoi012aa1n03x5               g111(.a(new_n202), .b(new_n197), .c(new_n199), .o1(new_n207));
  nor002aa1d24x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nand42aa1d28x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nanb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  nano22aa1n03x5               g115(.a(new_n207), .b(new_n206), .c(new_n210), .out0(new_n211));
  nand42aa1n02x5               g116(.a(new_n188), .b(new_n187), .o1(new_n212));
  oaoi03aa1n09x5               g117(.a(\a[18] ), .b(\b[17] ), .c(new_n212), .o1(new_n213));
  aoai13aa1n02x5               g118(.a(new_n203), .b(new_n213), .c(new_n184), .d(new_n196), .o1(new_n214));
  tech160nm_fiaoi012aa1n02p5x5 g119(.a(new_n210), .b(new_n214), .c(new_n206), .o1(new_n215));
  norp02aa1n03x5               g120(.a(new_n215), .b(new_n211), .o1(\s[20] ));
  nano23aa1n09x5               g121(.a(new_n200), .b(new_n208), .c(new_n209), .d(new_n201), .out0(new_n217));
  nand02aa1d04x5               g122(.a(new_n196), .b(new_n217), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  aoai13aa1n06x5               g124(.a(new_n219), .b(new_n195), .c(new_n192), .d(new_n180), .o1(new_n220));
  nona23aa1n09x5               g125(.a(new_n209), .b(new_n201), .c(new_n200), .d(new_n208), .out0(new_n221));
  aoi012aa1n12x5               g126(.a(new_n208), .b(new_n200), .c(new_n209), .o1(new_n222));
  oai012aa1n18x5               g127(.a(new_n222), .b(new_n221), .c(new_n199), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  nor002aa1d32x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  nanp02aa1n04x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n226), .b(new_n225), .out0(new_n227));
  xnbna2aa1n03x5               g132(.a(new_n227), .b(new_n220), .c(new_n224), .out0(\s[21] ));
  inv040aa1d28x5               g133(.a(new_n225), .o1(new_n229));
  aobi12aa1n03x5               g134(.a(new_n227), .b(new_n220), .c(new_n224), .out0(new_n230));
  xnrc02aa1n12x5               g135(.a(\b[21] ), .b(\a[22] ), .out0(new_n231));
  nano22aa1n02x4               g136(.a(new_n230), .b(new_n229), .c(new_n231), .out0(new_n232));
  aoai13aa1n03x5               g137(.a(new_n227), .b(new_n223), .c(new_n184), .d(new_n219), .o1(new_n233));
  tech160nm_fiaoi012aa1n02p5x5 g138(.a(new_n231), .b(new_n233), .c(new_n229), .o1(new_n234));
  norp02aa1n03x5               g139(.a(new_n234), .b(new_n232), .o1(\s[22] ));
  nano22aa1d15x5               g140(.a(new_n231), .b(new_n229), .c(new_n226), .out0(new_n236));
  and003aa1n02x5               g141(.a(new_n196), .b(new_n236), .c(new_n217), .o(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n195), .c(new_n192), .d(new_n180), .o1(new_n238));
  oao003aa1n09x5               g143(.a(\a[22] ), .b(\b[21] ), .c(new_n229), .carry(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  aoi012aa1n02x5               g145(.a(new_n240), .b(new_n223), .c(new_n236), .o1(new_n241));
  xnrc02aa1n12x5               g146(.a(\b[22] ), .b(\a[23] ), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  xnbna2aa1n03x5               g148(.a(new_n243), .b(new_n238), .c(new_n241), .out0(\s[23] ));
  nor042aa1n06x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  tech160nm_fiaoi012aa1n04x5   g151(.a(new_n242), .b(new_n238), .c(new_n241), .o1(new_n247));
  xnrc02aa1n03x5               g152(.a(\b[23] ), .b(\a[24] ), .out0(new_n248));
  nano22aa1n03x7               g153(.a(new_n247), .b(new_n246), .c(new_n248), .out0(new_n249));
  inv000aa1n02x5               g154(.a(new_n241), .o1(new_n250));
  aoai13aa1n03x5               g155(.a(new_n243), .b(new_n250), .c(new_n184), .d(new_n237), .o1(new_n251));
  aoi012aa1n03x5               g156(.a(new_n248), .b(new_n251), .c(new_n246), .o1(new_n252));
  norp02aa1n02x5               g157(.a(new_n252), .b(new_n249), .o1(\s[24] ));
  nor042aa1n06x5               g158(.a(new_n248), .b(new_n242), .o1(new_n254));
  nano22aa1n03x7               g159(.a(new_n218), .b(new_n236), .c(new_n254), .out0(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n195), .c(new_n192), .d(new_n180), .o1(new_n256));
  inv020aa1n04x5               g161(.a(new_n222), .o1(new_n257));
  aoai13aa1n06x5               g162(.a(new_n236), .b(new_n257), .c(new_n217), .d(new_n213), .o1(new_n258));
  inv040aa1n02x5               g163(.a(new_n254), .o1(new_n259));
  oao003aa1n03x5               g164(.a(\a[24] ), .b(\b[23] ), .c(new_n246), .carry(new_n260));
  aoai13aa1n12x5               g165(.a(new_n260), .b(new_n259), .c(new_n258), .d(new_n239), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  xnrc02aa1n12x5               g167(.a(\b[24] ), .b(\a[25] ), .out0(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  xnbna2aa1n03x5               g169(.a(new_n264), .b(new_n256), .c(new_n262), .out0(\s[25] ));
  nor042aa1n03x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  aoi012aa1n02x7               g172(.a(new_n263), .b(new_n256), .c(new_n262), .o1(new_n268));
  tech160nm_fixnrc02aa1n04x5   g173(.a(\b[25] ), .b(\a[26] ), .out0(new_n269));
  nano22aa1n03x5               g174(.a(new_n268), .b(new_n267), .c(new_n269), .out0(new_n270));
  aoai13aa1n03x5               g175(.a(new_n264), .b(new_n261), .c(new_n184), .d(new_n255), .o1(new_n271));
  aoi012aa1n03x5               g176(.a(new_n269), .b(new_n271), .c(new_n267), .o1(new_n272));
  nor002aa1n02x5               g177(.a(new_n272), .b(new_n270), .o1(\s[26] ));
  nor042aa1d18x5               g178(.a(new_n269), .b(new_n263), .o1(new_n274));
  nano32aa1d12x5               g179(.a(new_n218), .b(new_n274), .c(new_n236), .d(new_n254), .out0(new_n275));
  aoai13aa1n12x5               g180(.a(new_n275), .b(new_n195), .c(new_n192), .d(new_n180), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[26] ), .b(\b[25] ), .c(new_n267), .carry(new_n277));
  aobi12aa1n18x5               g182(.a(new_n277), .b(new_n261), .c(new_n274), .out0(new_n278));
  xorc02aa1n12x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  xnbna2aa1n03x5               g184(.a(new_n279), .b(new_n276), .c(new_n278), .out0(\s[27] ));
  norp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  inv040aa1n03x5               g186(.a(new_n281), .o1(new_n282));
  aobi12aa1n06x5               g187(.a(new_n279), .b(new_n276), .c(new_n278), .out0(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .out0(new_n284));
  nano22aa1n03x7               g189(.a(new_n283), .b(new_n282), .c(new_n284), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n254), .b(new_n240), .c(new_n223), .d(new_n236), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n274), .o1(new_n287));
  aoai13aa1n06x5               g192(.a(new_n277), .b(new_n287), .c(new_n286), .d(new_n260), .o1(new_n288));
  aoai13aa1n02x5               g193(.a(new_n279), .b(new_n288), .c(new_n184), .d(new_n275), .o1(new_n289));
  tech160nm_fiaoi012aa1n02p5x5 g194(.a(new_n284), .b(new_n289), .c(new_n282), .o1(new_n290));
  norp02aa1n03x5               g195(.a(new_n290), .b(new_n285), .o1(\s[28] ));
  norb02aa1n02x5               g196(.a(new_n279), .b(new_n284), .out0(new_n292));
  aoai13aa1n02x5               g197(.a(new_n292), .b(new_n288), .c(new_n184), .d(new_n275), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[28] ), .b(\a[29] ), .out0(new_n295));
  tech160nm_fiaoi012aa1n02p5x5 g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  aobi12aa1n06x5               g201(.a(new_n292), .b(new_n276), .c(new_n278), .out0(new_n297));
  nano22aa1n03x7               g202(.a(new_n297), .b(new_n294), .c(new_n295), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n296), .b(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g205(.a(new_n279), .b(new_n295), .c(new_n284), .out0(new_n301));
  aoai13aa1n02x5               g206(.a(new_n301), .b(new_n288), .c(new_n184), .d(new_n275), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[29] ), .b(\a[30] ), .out0(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n304), .b(new_n302), .c(new_n303), .o1(new_n305));
  aobi12aa1n06x5               g210(.a(new_n301), .b(new_n276), .c(new_n278), .out0(new_n306));
  nano22aa1n03x7               g211(.a(new_n306), .b(new_n303), .c(new_n304), .out0(new_n307));
  norp02aa1n03x5               g212(.a(new_n305), .b(new_n307), .o1(\s[30] ));
  xnrc02aa1n02x5               g213(.a(\b[30] ), .b(\a[31] ), .out0(new_n309));
  norb02aa1n02x5               g214(.a(new_n301), .b(new_n304), .out0(new_n310));
  aobi12aa1n06x5               g215(.a(new_n310), .b(new_n276), .c(new_n278), .out0(new_n311));
  oao003aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n303), .carry(new_n312));
  nano22aa1n03x7               g217(.a(new_n311), .b(new_n309), .c(new_n312), .out0(new_n313));
  aoai13aa1n02x5               g218(.a(new_n310), .b(new_n288), .c(new_n184), .d(new_n275), .o1(new_n314));
  tech160nm_fiaoi012aa1n02p5x5 g219(.a(new_n309), .b(new_n314), .c(new_n312), .o1(new_n315));
  norp02aa1n03x5               g220(.a(new_n315), .b(new_n313), .o1(\s[31] ));
  xnrb03aa1n02x5               g221(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g222(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g224(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g225(.a(new_n118), .b(new_n119), .c(new_n109), .o1(new_n321));
  xnrb03aa1n02x5               g226(.a(new_n321), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norp02aa1n02x5               g227(.a(new_n116), .b(new_n115), .o1(new_n323));
  aobi12aa1n02x5               g228(.a(new_n121), .b(new_n109), .c(new_n323), .out0(new_n324));
  xnrb03aa1n02x5               g229(.a(new_n324), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g230(.a(\a[7] ), .b(\b[6] ), .c(new_n324), .o1(new_n326));
  xorb03aa1n02x5               g231(.a(new_n326), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g232(.a(new_n192), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


