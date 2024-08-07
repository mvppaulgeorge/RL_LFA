// Benchmark "adder" written by ABC on Thu Jul 18 05:59:46 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n323, new_n325, new_n326, new_n328,
    new_n329, new_n330, new_n331, new_n332;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nand02aa1n03x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  oaoi03aa1n09x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor022aa1n06x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand02aa1d16x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor002aa1d32x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand42aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n06x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  tech160nm_fiaoi012aa1n03p5x5 g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nor042aa1n06x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n04x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n04x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n112), .b(new_n111), .c(new_n113), .d(new_n110), .out0(new_n114));
  xorc02aa1n02x5               g019(.a(\a[5] ), .b(\b[4] ), .out0(new_n115));
  xorc02aa1n12x5               g020(.a(\a[6] ), .b(\b[5] ), .out0(new_n116));
  nano22aa1n03x7               g021(.a(new_n114), .b(new_n115), .c(new_n116), .out0(new_n117));
  tech160nm_fioai012aa1n03p5x5 g022(.a(new_n111), .b(new_n113), .c(new_n110), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  oai022aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  oai012aa1n12x5               g026(.a(new_n118), .b(new_n114), .c(new_n121), .o1(new_n122));
  nand42aa1n08x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  norb02aa1n02x5               g028(.a(new_n123), .b(new_n97), .out0(new_n124));
  aoai13aa1n06x5               g029(.a(new_n124), .b(new_n122), .c(new_n117), .d(new_n109), .o1(new_n125));
  nor002aa1d32x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand42aa1d28x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g034(.a(new_n126), .o1(new_n130));
  inv000aa1d42x5               g035(.a(new_n127), .o1(new_n131));
  aoai13aa1n06x5               g036(.a(new_n130), .b(new_n131), .c(new_n125), .d(new_n98), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1d18x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand22aa1n12x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanb02aa1d24x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  inv040aa1n09x5               g041(.a(new_n136), .o1(new_n137));
  xorc02aa1n12x5               g042(.a(\a[12] ), .b(\b[11] ), .out0(new_n138));
  aoi112aa1n02x5               g043(.a(new_n138), .b(new_n134), .c(new_n132), .d(new_n137), .o1(new_n139));
  aoai13aa1n02x5               g044(.a(new_n138), .b(new_n134), .c(new_n132), .d(new_n137), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(\s[12] ));
  nano23aa1d15x5               g046(.a(new_n97), .b(new_n126), .c(new_n127), .d(new_n123), .out0(new_n142));
  nand23aa1d12x5               g047(.a(new_n142), .b(new_n137), .c(new_n138), .o1(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  aoai13aa1n06x5               g049(.a(new_n144), .b(new_n122), .c(new_n117), .d(new_n109), .o1(new_n145));
  xnrc02aa1n12x5               g050(.a(\b[11] ), .b(\a[12] ), .out0(new_n146));
  inv040aa1n03x5               g051(.a(new_n134), .o1(new_n147));
  oao003aa1n09x5               g052(.a(\a[12] ), .b(\b[11] ), .c(new_n147), .carry(new_n148));
  oai012aa1n12x5               g053(.a(new_n127), .b(new_n126), .c(new_n97), .o1(new_n149));
  oai013aa1d12x5               g054(.a(new_n148), .b(new_n146), .c(new_n149), .d(new_n136), .o1(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  nor002aa1d32x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand42aa1n04x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  norb02aa1n06x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n145), .c(new_n151), .out0(\s[13] ));
  inv000aa1d42x5               g060(.a(new_n152), .o1(new_n156));
  oao003aa1n02x5               g061(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n157));
  nano23aa1n03x5               g062(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n158));
  aobi12aa1n03x7               g063(.a(new_n108), .b(new_n158), .c(new_n157), .out0(new_n159));
  nano23aa1n02x4               g064(.a(new_n113), .b(new_n110), .c(new_n111), .d(new_n112), .out0(new_n160));
  nanp03aa1n02x5               g065(.a(new_n160), .b(new_n115), .c(new_n116), .o1(new_n161));
  oabi12aa1n06x5               g066(.a(new_n122), .b(new_n159), .c(new_n161), .out0(new_n162));
  aoai13aa1n02x5               g067(.a(new_n154), .b(new_n150), .c(new_n162), .d(new_n144), .o1(new_n163));
  nor042aa1n04x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nanp02aa1n04x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  norb02aa1n03x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  xnbna2aa1n03x5               g071(.a(new_n166), .b(new_n163), .c(new_n156), .out0(\s[14] ));
  oaih12aa1n02x5               g072(.a(new_n165), .b(new_n164), .c(new_n152), .o1(new_n168));
  nona23aa1n09x5               g073(.a(new_n165), .b(new_n153), .c(new_n152), .d(new_n164), .out0(new_n169));
  aoai13aa1n06x5               g074(.a(new_n168), .b(new_n169), .c(new_n145), .d(new_n151), .o1(new_n170));
  xorb03aa1n02x5               g075(.a(new_n170), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nanp02aa1n04x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nor002aa1d32x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand42aa1n06x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nanb02aa1n12x5               g080(.a(new_n174), .b(new_n175), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  aoi112aa1n02x5               g082(.a(new_n177), .b(new_n172), .c(new_n170), .d(new_n173), .o1(new_n178));
  aoai13aa1n03x5               g083(.a(new_n177), .b(new_n172), .c(new_n170), .d(new_n173), .o1(new_n179));
  norb02aa1n03x4               g084(.a(new_n179), .b(new_n178), .out0(\s[16] ));
  nano23aa1n06x5               g085(.a(new_n172), .b(new_n174), .c(new_n175), .d(new_n173), .out0(new_n181));
  nano32aa1d12x5               g086(.a(new_n143), .b(new_n181), .c(new_n154), .d(new_n166), .out0(new_n182));
  aoai13aa1n12x5               g087(.a(new_n182), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n183));
  nanb02aa1n02x5               g088(.a(new_n172), .b(new_n173), .out0(new_n184));
  norp03aa1n06x5               g089(.a(new_n169), .b(new_n176), .c(new_n184), .o1(new_n185));
  norp03aa1n06x5               g090(.a(new_n168), .b(new_n176), .c(new_n184), .o1(new_n186));
  tech160nm_fiao0012aa1n02p5x5 g091(.a(new_n174), .b(new_n172), .c(new_n175), .o(new_n187));
  aoi112aa1n09x5               g092(.a(new_n187), .b(new_n186), .c(new_n150), .d(new_n185), .o1(new_n188));
  xorc02aa1n12x5               g093(.a(\a[17] ), .b(\b[16] ), .out0(new_n189));
  xnbna2aa1n03x5               g094(.a(new_n189), .b(new_n183), .c(new_n188), .out0(\s[17] ));
  inv000aa1d42x5               g095(.a(\a[17] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(\b[16] ), .o1(new_n192));
  nanp02aa1n09x5               g097(.a(new_n183), .b(new_n188), .o1(new_n193));
  tech160nm_fioaoi03aa1n03p5x5 g098(.a(new_n191), .b(new_n192), .c(new_n193), .o1(new_n194));
  norp02aa1n06x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  nand42aa1n03x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  nanb02aa1n06x5               g101(.a(new_n195), .b(new_n196), .out0(new_n197));
  tech160nm_fixorc02aa1n02p5x5 g102(.a(new_n194), .b(new_n197), .out0(\s[18] ));
  norb02aa1n03x5               g103(.a(new_n189), .b(new_n197), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  aoai13aa1n04x5               g105(.a(new_n196), .b(new_n195), .c(new_n191), .d(new_n192), .o1(new_n201));
  aoai13aa1n04x5               g106(.a(new_n201), .b(new_n200), .c(new_n183), .d(new_n188), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nand42aa1n06x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  inv000aa1d42x5               g111(.a(\b[19] ), .o1(new_n207));
  nanb02aa1n12x5               g112(.a(\a[20] ), .b(new_n207), .out0(new_n208));
  nanp02aa1n04x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  aoi122aa1n03x5               g114(.a(new_n205), .b(new_n208), .c(new_n209), .d(new_n202), .e(new_n206), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n205), .o1(new_n211));
  nanb02aa1n12x5               g116(.a(new_n205), .b(new_n206), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  nand02aa1n02x5               g118(.a(new_n202), .b(new_n213), .o1(new_n214));
  nanp02aa1n03x5               g119(.a(new_n208), .b(new_n209), .o1(new_n215));
  tech160nm_fiaoi012aa1n02p5x5 g120(.a(new_n215), .b(new_n214), .c(new_n211), .o1(new_n216));
  norp02aa1n03x5               g121(.a(new_n216), .b(new_n210), .o1(\s[20] ));
  nona23aa1n02x4               g122(.a(new_n213), .b(new_n189), .c(new_n197), .d(new_n215), .out0(new_n218));
  nand42aa1n03x5               g123(.a(new_n205), .b(new_n209), .o1(new_n219));
  nor043aa1n03x5               g124(.a(new_n201), .b(new_n212), .c(new_n215), .o1(new_n220));
  nano22aa1n03x7               g125(.a(new_n220), .b(new_n208), .c(new_n219), .out0(new_n221));
  aoai13aa1n04x5               g126(.a(new_n221), .b(new_n218), .c(new_n183), .d(new_n188), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[20] ), .b(\a[21] ), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  xnrc02aa1n12x5               g131(.a(\b[21] ), .b(\a[22] ), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoi112aa1n02x5               g133(.a(new_n224), .b(new_n228), .c(new_n222), .d(new_n226), .o1(new_n229));
  aoai13aa1n03x5               g134(.a(new_n228), .b(new_n224), .c(new_n222), .d(new_n226), .o1(new_n230));
  norb02aa1n03x4               g135(.a(new_n230), .b(new_n229), .out0(\s[22] ));
  norp02aa1n02x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  nona23aa1n09x5               g137(.a(new_n209), .b(new_n206), .c(new_n205), .d(new_n232), .out0(new_n233));
  nor042aa1n06x5               g138(.a(new_n227), .b(new_n225), .o1(new_n234));
  nona23aa1n08x5               g139(.a(new_n234), .b(new_n189), .c(new_n233), .d(new_n197), .out0(new_n235));
  oai112aa1n04x5               g140(.a(new_n219), .b(new_n208), .c(new_n233), .d(new_n201), .o1(new_n236));
  inv020aa1n02x5               g141(.a(new_n224), .o1(new_n237));
  oaoi03aa1n06x5               g142(.a(\a[22] ), .b(\b[21] ), .c(new_n237), .o1(new_n238));
  aoi012aa1n02x5               g143(.a(new_n238), .b(new_n236), .c(new_n234), .o1(new_n239));
  aoai13aa1n04x5               g144(.a(new_n239), .b(new_n235), .c(new_n183), .d(new_n188), .o1(new_n240));
  xorb03aa1n02x5               g145(.a(new_n240), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  xorc02aa1n12x5               g147(.a(\a[23] ), .b(\b[22] ), .out0(new_n243));
  xorc02aa1n12x5               g148(.a(\a[24] ), .b(\b[23] ), .out0(new_n244));
  aoi112aa1n02x7               g149(.a(new_n242), .b(new_n244), .c(new_n240), .d(new_n243), .o1(new_n245));
  aoai13aa1n03x5               g150(.a(new_n244), .b(new_n242), .c(new_n240), .d(new_n243), .o1(new_n246));
  norb02aa1n02x7               g151(.a(new_n246), .b(new_n245), .out0(\s[24] ));
  nanp02aa1n02x5               g152(.a(new_n244), .b(new_n243), .o1(new_n248));
  nona23aa1n06x5               g153(.a(new_n234), .b(new_n199), .c(new_n248), .d(new_n233), .out0(new_n249));
  xnrc02aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .out0(new_n250));
  norb02aa1n02x5               g155(.a(new_n244), .b(new_n250), .out0(new_n251));
  norp02aa1n02x5               g156(.a(\b[23] ), .b(\a[24] ), .o1(new_n252));
  aoi112aa1n02x5               g157(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n253));
  nanp03aa1n03x5               g158(.a(new_n238), .b(new_n243), .c(new_n244), .o1(new_n254));
  nona22aa1n09x5               g159(.a(new_n254), .b(new_n253), .c(new_n252), .out0(new_n255));
  aoi013aa1n02x4               g160(.a(new_n255), .b(new_n236), .c(new_n234), .d(new_n251), .o1(new_n256));
  aoai13aa1n04x5               g161(.a(new_n256), .b(new_n249), .c(new_n183), .d(new_n188), .o1(new_n257));
  xorb03aa1n02x5               g162(.a(new_n257), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor002aa1n03x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  tech160nm_fixorc02aa1n05x5   g164(.a(\a[25] ), .b(\b[24] ), .out0(new_n260));
  tech160nm_fixorc02aa1n05x5   g165(.a(\a[26] ), .b(\b[25] ), .out0(new_n261));
  aoi112aa1n02x5               g166(.a(new_n259), .b(new_n261), .c(new_n257), .d(new_n260), .o1(new_n262));
  aoai13aa1n03x5               g167(.a(new_n261), .b(new_n259), .c(new_n257), .d(new_n260), .o1(new_n263));
  norb02aa1n03x4               g168(.a(new_n263), .b(new_n262), .out0(\s[26] ));
  nanp02aa1n02x5               g169(.a(new_n150), .b(new_n185), .o1(new_n265));
  nona22aa1n02x4               g170(.a(new_n265), .b(new_n186), .c(new_n187), .out0(new_n266));
  and002aa1n18x5               g171(.a(new_n261), .b(new_n260), .o(new_n267));
  nano22aa1n03x7               g172(.a(new_n235), .b(new_n251), .c(new_n267), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n266), .c(new_n162), .d(new_n182), .o1(new_n269));
  nano22aa1n03x5               g174(.a(new_n221), .b(new_n234), .c(new_n251), .out0(new_n270));
  inv040aa1d30x5               g175(.a(\a[26] ), .o1(new_n271));
  inv000aa1d42x5               g176(.a(\b[25] ), .o1(new_n272));
  tech160nm_fioaoi03aa1n03p5x5 g177(.a(new_n271), .b(new_n272), .c(new_n259), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  oaoi13aa1n09x5               g179(.a(new_n274), .b(new_n267), .c(new_n270), .d(new_n255), .o1(new_n275));
  xorc02aa1n12x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  xnbna2aa1n03x5               g181(.a(new_n276), .b(new_n275), .c(new_n269), .out0(\s[27] ));
  norp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  inv040aa1n03x5               g183(.a(new_n278), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n276), .o1(new_n280));
  tech160nm_fiaoi012aa1n03p5x5 g185(.a(new_n280), .b(new_n275), .c(new_n269), .o1(new_n281));
  xnrc02aa1n12x5               g186(.a(\b[27] ), .b(\a[28] ), .out0(new_n282));
  nano22aa1n02x4               g187(.a(new_n281), .b(new_n279), .c(new_n282), .out0(new_n283));
  nona32aa1n02x5               g188(.a(new_n236), .b(new_n248), .c(new_n227), .d(new_n225), .out0(new_n284));
  inv000aa1n02x5               g189(.a(new_n255), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n267), .o1(new_n286));
  aoai13aa1n06x5               g191(.a(new_n273), .b(new_n286), .c(new_n284), .d(new_n285), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n276), .b(new_n287), .c(new_n193), .d(new_n268), .o1(new_n288));
  tech160nm_fiaoi012aa1n05x5   g193(.a(new_n282), .b(new_n288), .c(new_n279), .o1(new_n289));
  norp02aa1n03x5               g194(.a(new_n289), .b(new_n283), .o1(\s[28] ));
  xnrc02aa1n02x5               g195(.a(\b[28] ), .b(\a[29] ), .out0(new_n291));
  norb02aa1d21x5               g196(.a(new_n276), .b(new_n282), .out0(new_n292));
  aoai13aa1n03x5               g197(.a(new_n292), .b(new_n287), .c(new_n193), .d(new_n268), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .c(new_n279), .carry(new_n294));
  aoi012aa1n03x5               g199(.a(new_n291), .b(new_n293), .c(new_n294), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n292), .o1(new_n296));
  tech160nm_fiaoi012aa1n02p5x5 g201(.a(new_n296), .b(new_n275), .c(new_n269), .o1(new_n297));
  nano22aa1n03x5               g202(.a(new_n297), .b(new_n291), .c(new_n294), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n295), .b(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g205(.a(new_n276), .b(new_n291), .c(new_n282), .out0(new_n301));
  aoai13aa1n03x5               g206(.a(new_n301), .b(new_n287), .c(new_n193), .d(new_n268), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[29] ), .b(\a[30] ), .out0(new_n304));
  aoi012aa1n03x5               g209(.a(new_n304), .b(new_n302), .c(new_n303), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n301), .o1(new_n306));
  tech160nm_fiaoi012aa1n02p5x5 g211(.a(new_n306), .b(new_n275), .c(new_n269), .o1(new_n307));
  nano22aa1n02x4               g212(.a(new_n307), .b(new_n303), .c(new_n304), .out0(new_n308));
  norp02aa1n03x5               g213(.a(new_n305), .b(new_n308), .o1(\s[30] ));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  norb02aa1n03x5               g215(.a(new_n301), .b(new_n304), .out0(new_n311));
  inv020aa1n02x5               g216(.a(new_n311), .o1(new_n312));
  tech160nm_fiaoi012aa1n02p5x5 g217(.a(new_n312), .b(new_n275), .c(new_n269), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n303), .carry(new_n314));
  nano22aa1n03x5               g219(.a(new_n313), .b(new_n310), .c(new_n314), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n311), .b(new_n287), .c(new_n193), .d(new_n268), .o1(new_n316));
  aoi012aa1n03x5               g221(.a(new_n310), .b(new_n316), .c(new_n314), .o1(new_n317));
  norp02aa1n03x5               g222(.a(new_n317), .b(new_n315), .o1(\s[31] ));
  xnrb03aa1n02x5               g223(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g224(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g226(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oao003aa1n03x5               g227(.a(\a[5] ), .b(\b[4] ), .c(new_n159), .carry(new_n323));
  xnrc02aa1n02x5               g228(.a(new_n323), .b(new_n116), .out0(\s[6] ));
  nanb02aa1n02x5               g229(.a(new_n113), .b(new_n112), .out0(new_n325));
  nanp02aa1n02x5               g230(.a(new_n323), .b(new_n116), .o1(new_n326));
  xnbna2aa1n03x5               g231(.a(new_n325), .b(new_n326), .c(new_n119), .out0(\s[7] ));
  nanb02aa1n02x5               g232(.a(new_n110), .b(new_n111), .out0(new_n328));
  inv000aa1d42x5               g233(.a(new_n113), .o1(new_n329));
  nanp03aa1n02x5               g234(.a(new_n326), .b(new_n112), .c(new_n119), .o1(new_n330));
  tech160nm_fiaoi012aa1n03p5x5 g235(.a(new_n328), .b(new_n330), .c(new_n329), .o1(new_n331));
  nanp03aa1n02x5               g236(.a(new_n330), .b(new_n328), .c(new_n329), .o1(new_n332));
  norb02aa1n02x5               g237(.a(new_n332), .b(new_n331), .out0(\s[8] ));
  xorb03aa1n02x5               g238(.a(new_n162), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


