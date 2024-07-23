// Benchmark "adder" written by ABC on Thu Jul 18 09:28:18 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n312, new_n314, new_n316, new_n318, new_n320;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  and002aa1n03x5               g002(.a(\b[8] ), .b(\a[9] ), .o(new_n98));
  nor002aa1n06x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  nor002aa1n08x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nand42aa1n16x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nor002aa1n08x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nand02aa1n08x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nano23aa1n09x5               g008(.a(new_n100), .b(new_n102), .c(new_n103), .d(new_n101), .out0(new_n104));
  nand42aa1n20x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nor022aa1n16x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  nor002aa1d32x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  oa0012aa1n03x5               g012(.a(new_n105), .b(new_n106), .c(new_n107), .o(new_n108));
  oa0012aa1n02x5               g013(.a(new_n101), .b(new_n102), .c(new_n100), .o(new_n109));
  aoi012aa1n06x5               g014(.a(new_n109), .b(new_n104), .c(new_n108), .o1(new_n110));
  nor042aa1n04x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nanp02aa1n09x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  norb02aa1n06x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  nor042aa1n03x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nanp02aa1n04x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  norb02aa1n03x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  and002aa1n12x5               g021(.a(\b[1] ), .b(\a[2] ), .o(new_n117));
  nand22aa1n12x5               g022(.a(\b[0] ), .b(\a[1] ), .o1(new_n118));
  norp02aa1n24x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  oab012aa1d15x5               g024(.a(new_n117), .b(new_n119), .c(new_n118), .out0(new_n120));
  nanp03aa1d12x5               g025(.a(new_n120), .b(new_n113), .c(new_n116), .o1(new_n121));
  tech160nm_fioai012aa1n03p5x5 g026(.a(new_n112), .b(new_n114), .c(new_n111), .o1(new_n122));
  nand42aa1n06x5               g027(.a(\b[4] ), .b(\a[5] ), .o1(new_n123));
  nano23aa1n03x7               g028(.a(new_n107), .b(new_n106), .c(new_n123), .d(new_n105), .out0(new_n124));
  nanp02aa1n03x5               g029(.a(new_n124), .b(new_n104), .o1(new_n125));
  aoai13aa1n12x5               g030(.a(new_n110), .b(new_n125), .c(new_n121), .d(new_n122), .o1(new_n126));
  oabi12aa1n06x5               g031(.a(new_n98), .b(new_n126), .c(new_n99), .out0(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  and002aa1n12x5               g033(.a(\b[9] ), .b(\a[10] ), .o(new_n129));
  xorc02aa1n12x5               g034(.a(\a[10] ), .b(\b[9] ), .out0(new_n130));
  nor042aa1n06x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand42aa1n20x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n03x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  aoi112aa1n03x5               g039(.a(new_n134), .b(new_n129), .c(new_n127), .d(new_n130), .o1(new_n135));
  aoai13aa1n02x5               g040(.a(new_n134), .b(new_n129), .c(new_n127), .d(new_n130), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(\s[11] ));
  nor042aa1n04x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand42aa1d28x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(new_n140));
  oai012aa1n03x5               g045(.a(new_n140), .b(new_n135), .c(new_n131), .o1(new_n141));
  nanp02aa1n03x5               g046(.a(new_n127), .b(new_n130), .o1(new_n142));
  nona22aa1n02x4               g047(.a(new_n142), .b(new_n134), .c(new_n129), .out0(new_n143));
  nona22aa1n02x4               g048(.a(new_n143), .b(new_n140), .c(new_n131), .out0(new_n144));
  nanp02aa1n03x5               g049(.a(new_n141), .b(new_n144), .o1(\s[12] ));
  nano23aa1n09x5               g050(.a(new_n131), .b(new_n138), .c(new_n139), .d(new_n132), .out0(new_n146));
  norp02aa1n04x5               g051(.a(\b[9] ), .b(\a[10] ), .o1(new_n147));
  oab012aa1n09x5               g052(.a(new_n129), .b(new_n147), .c(new_n99), .out0(new_n148));
  oai022aa1n02x5               g053(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n149));
  aoi022aa1n09x5               g054(.a(new_n146), .b(new_n148), .c(new_n139), .d(new_n149), .o1(new_n150));
  nor002aa1n02x5               g055(.a(new_n98), .b(new_n99), .o1(new_n151));
  nano32aa1n03x5               g056(.a(new_n140), .b(new_n130), .c(new_n151), .d(new_n133), .out0(new_n152));
  nanp02aa1n06x5               g057(.a(new_n126), .b(new_n152), .o1(new_n153));
  xorc02aa1n12x5               g058(.a(\a[13] ), .b(\b[12] ), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n153), .c(new_n150), .out0(\s[13] ));
  inv000aa1d42x5               g060(.a(\a[14] ), .o1(new_n156));
  nanp02aa1n03x5               g061(.a(new_n153), .b(new_n150), .o1(new_n157));
  nor042aa1n06x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  tech160nm_fiaoi012aa1n05x5   g063(.a(new_n158), .b(new_n157), .c(new_n154), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(new_n156), .out0(\s[14] ));
  xnrc02aa1n12x5               g065(.a(\b[13] ), .b(\a[14] ), .out0(new_n161));
  nanb02aa1n02x5               g066(.a(new_n161), .b(new_n154), .out0(new_n162));
  inv000aa1d42x5               g067(.a(\b[13] ), .o1(new_n163));
  tech160nm_fioaoi03aa1n03p5x5 g068(.a(new_n156), .b(new_n163), .c(new_n158), .o1(new_n164));
  aoai13aa1n06x5               g069(.a(new_n164), .b(new_n162), .c(new_n153), .d(new_n150), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1d18x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nand22aa1n06x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  norb02aa1n06x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  nor042aa1d18x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanp02aa1n12x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanb02aa1n12x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  aoai13aa1n03x5               g077(.a(new_n172), .b(new_n167), .c(new_n165), .d(new_n169), .o1(new_n173));
  nand42aa1n02x5               g078(.a(new_n165), .b(new_n169), .o1(new_n174));
  nona22aa1n03x5               g079(.a(new_n174), .b(new_n172), .c(new_n167), .out0(new_n175));
  nanp02aa1n03x5               g080(.a(new_n175), .b(new_n173), .o1(\s[16] ));
  nona23aa1n03x5               g081(.a(new_n171), .b(new_n168), .c(new_n167), .d(new_n170), .out0(new_n177));
  oai012aa1n02x7               g082(.a(new_n171), .b(new_n170), .c(new_n167), .o1(new_n178));
  oaih12aa1n02x5               g083(.a(new_n178), .b(new_n177), .c(new_n164), .o1(new_n179));
  nona23aa1d18x5               g084(.a(new_n154), .b(new_n169), .c(new_n161), .d(new_n172), .out0(new_n180));
  oab012aa1n09x5               g085(.a(new_n179), .b(new_n180), .c(new_n150), .out0(new_n181));
  nano32aa1d12x5               g086(.a(new_n180), .b(new_n151), .c(new_n146), .d(new_n130), .out0(new_n182));
  nand02aa1d08x5               g087(.a(new_n126), .b(new_n182), .o1(new_n183));
  xorc02aa1n02x5               g088(.a(\a[17] ), .b(\b[16] ), .out0(new_n184));
  xnbna2aa1n03x5               g089(.a(new_n184), .b(new_n183), .c(new_n181), .out0(\s[17] ));
  inv000aa1d42x5               g090(.a(\a[18] ), .o1(new_n186));
  nanp02aa1n06x5               g091(.a(new_n183), .b(new_n181), .o1(new_n187));
  nor002aa1n12x5               g092(.a(\b[16] ), .b(\a[17] ), .o1(new_n188));
  tech160nm_fiaoi012aa1n05x5   g093(.a(new_n188), .b(new_n187), .c(new_n184), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[17] ), .c(new_n186), .out0(\s[18] ));
  oabi12aa1n09x5               g095(.a(new_n179), .b(new_n180), .c(new_n150), .out0(new_n191));
  inv000aa1d42x5               g096(.a(\a[17] ), .o1(new_n192));
  xroi22aa1d06x4               g097(.a(new_n192), .b(\b[16] ), .c(new_n186), .d(\b[17] ), .out0(new_n193));
  aoai13aa1n06x5               g098(.a(new_n193), .b(new_n191), .c(new_n126), .d(new_n182), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[17] ), .o1(new_n195));
  oaoi03aa1n12x5               g100(.a(new_n186), .b(new_n195), .c(new_n188), .o1(new_n196));
  nor002aa1d32x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nand02aa1d10x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  xnbna2aa1n03x5               g105(.a(new_n200), .b(new_n194), .c(new_n196), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand22aa1n03x5               g107(.a(new_n194), .b(new_n196), .o1(new_n203));
  nor002aa1d32x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nand02aa1d10x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nanb02aa1n02x5               g110(.a(new_n204), .b(new_n205), .out0(new_n206));
  aoai13aa1n03x5               g111(.a(new_n206), .b(new_n197), .c(new_n203), .d(new_n200), .o1(new_n207));
  nanp02aa1n02x5               g112(.a(new_n203), .b(new_n200), .o1(new_n208));
  nona22aa1n02x4               g113(.a(new_n208), .b(new_n206), .c(new_n197), .out0(new_n209));
  nanp02aa1n03x5               g114(.a(new_n209), .b(new_n207), .o1(\s[20] ));
  nona23aa1d18x5               g115(.a(new_n205), .b(new_n198), .c(new_n197), .d(new_n204), .out0(new_n211));
  oa0012aa1n12x5               g116(.a(new_n205), .b(new_n204), .c(new_n197), .o(new_n212));
  inv040aa1n08x5               g117(.a(new_n212), .o1(new_n213));
  oai012aa1d24x5               g118(.a(new_n213), .b(new_n211), .c(new_n196), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  nano23aa1n09x5               g120(.a(new_n197), .b(new_n204), .c(new_n205), .d(new_n198), .out0(new_n216));
  nanp02aa1n02x5               g121(.a(new_n193), .b(new_n216), .o1(new_n217));
  aoai13aa1n06x5               g122(.a(new_n215), .b(new_n217), .c(new_n183), .d(new_n181), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  xorc02aa1n02x5               g125(.a(\a[21] ), .b(\b[20] ), .out0(new_n221));
  xorc02aa1n02x5               g126(.a(\a[22] ), .b(\b[21] ), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoai13aa1n03x5               g128(.a(new_n223), .b(new_n220), .c(new_n218), .d(new_n221), .o1(new_n224));
  nand42aa1n02x5               g129(.a(new_n218), .b(new_n221), .o1(new_n225));
  nona22aa1n03x5               g130(.a(new_n225), .b(new_n223), .c(new_n220), .out0(new_n226));
  nanp02aa1n03x5               g131(.a(new_n226), .b(new_n224), .o1(\s[22] ));
  inv000aa1d42x5               g132(.a(\a[21] ), .o1(new_n228));
  inv040aa1d32x5               g133(.a(\a[22] ), .o1(new_n229));
  xroi22aa1d06x4               g134(.a(new_n228), .b(\b[20] ), .c(new_n229), .d(\b[21] ), .out0(new_n230));
  inv000aa1d42x5               g135(.a(\b[21] ), .o1(new_n231));
  oao003aa1n12x5               g136(.a(new_n229), .b(new_n231), .c(new_n220), .carry(new_n232));
  aoi012aa1n02x5               g137(.a(new_n232), .b(new_n214), .c(new_n230), .o1(new_n233));
  nand03aa1n02x5               g138(.a(new_n230), .b(new_n193), .c(new_n216), .o1(new_n234));
  aoai13aa1n06x5               g139(.a(new_n233), .b(new_n234), .c(new_n183), .d(new_n181), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  xorc02aa1n12x5               g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  tech160nm_fixnrc02aa1n05x5   g143(.a(\b[23] ), .b(\a[24] ), .out0(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n237), .c(new_n235), .d(new_n238), .o1(new_n240));
  nand42aa1n02x5               g145(.a(new_n235), .b(new_n238), .o1(new_n241));
  nona22aa1n02x4               g146(.a(new_n241), .b(new_n239), .c(new_n237), .out0(new_n242));
  nanp02aa1n03x5               g147(.a(new_n242), .b(new_n240), .o1(\s[24] ));
  norb02aa1n03x4               g148(.a(new_n238), .b(new_n239), .out0(new_n244));
  nano22aa1n03x5               g149(.a(new_n217), .b(new_n244), .c(new_n230), .out0(new_n245));
  aoai13aa1n03x5               g150(.a(new_n245), .b(new_n191), .c(new_n126), .d(new_n182), .o1(new_n246));
  oao003aa1n03x5               g151(.a(new_n186), .b(new_n195), .c(new_n188), .carry(new_n247));
  aoai13aa1n06x5               g152(.a(new_n230), .b(new_n212), .c(new_n216), .d(new_n247), .o1(new_n248));
  inv000aa1n02x5               g153(.a(new_n232), .o1(new_n249));
  inv030aa1n02x5               g154(.a(new_n244), .o1(new_n250));
  orn002aa1n03x5               g155(.a(\a[23] ), .b(\b[22] ), .o(new_n251));
  oaoi03aa1n02x5               g156(.a(\a[24] ), .b(\b[23] ), .c(new_n251), .o1(new_n252));
  inv030aa1n02x5               g157(.a(new_n252), .o1(new_n253));
  aoai13aa1n04x5               g158(.a(new_n253), .b(new_n250), .c(new_n248), .d(new_n249), .o1(new_n254));
  nanb02aa1n03x5               g159(.a(new_n254), .b(new_n246), .out0(new_n255));
  xorb03aa1n02x5               g160(.a(new_n255), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  xorc02aa1n12x5               g162(.a(\a[25] ), .b(\b[24] ), .out0(new_n258));
  nor042aa1n03x5               g163(.a(\b[25] ), .b(\a[26] ), .o1(new_n259));
  nand02aa1d16x5               g164(.a(\b[25] ), .b(\a[26] ), .o1(new_n260));
  norb02aa1n03x5               g165(.a(new_n260), .b(new_n259), .out0(new_n261));
  inv000aa1n04x5               g166(.a(new_n261), .o1(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n257), .c(new_n255), .d(new_n258), .o1(new_n263));
  aoai13aa1n06x5               g168(.a(new_n258), .b(new_n254), .c(new_n187), .d(new_n245), .o1(new_n264));
  nona22aa1n03x5               g169(.a(new_n264), .b(new_n262), .c(new_n257), .out0(new_n265));
  nanp02aa1n03x5               g170(.a(new_n263), .b(new_n265), .o1(\s[26] ));
  norb02aa1n06x5               g171(.a(new_n258), .b(new_n262), .out0(new_n267));
  nanp02aa1n02x5               g172(.a(new_n254), .b(new_n267), .o1(new_n268));
  oai012aa1n02x5               g173(.a(new_n260), .b(new_n259), .c(new_n257), .o1(new_n269));
  nano22aa1n03x7               g174(.a(new_n234), .b(new_n244), .c(new_n267), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n191), .c(new_n126), .d(new_n182), .o1(new_n271));
  nand23aa1n04x5               g176(.a(new_n268), .b(new_n271), .c(new_n269), .o1(new_n272));
  xorb03aa1n02x5               g177(.a(new_n272), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  xorc02aa1n02x5               g179(.a(\a[27] ), .b(\b[26] ), .out0(new_n275));
  xnrc02aa1n02x5               g180(.a(\b[27] ), .b(\a[28] ), .out0(new_n276));
  aoai13aa1n03x5               g181(.a(new_n276), .b(new_n274), .c(new_n272), .d(new_n275), .o1(new_n277));
  aoai13aa1n03x5               g182(.a(new_n244), .b(new_n232), .c(new_n214), .d(new_n230), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n267), .o1(new_n279));
  aoai13aa1n06x5               g184(.a(new_n269), .b(new_n279), .c(new_n278), .d(new_n253), .o1(new_n280));
  aobi12aa1n12x5               g185(.a(new_n270), .b(new_n183), .c(new_n181), .out0(new_n281));
  oaih12aa1n02x5               g186(.a(new_n275), .b(new_n280), .c(new_n281), .o1(new_n282));
  nona22aa1n03x5               g187(.a(new_n282), .b(new_n276), .c(new_n274), .out0(new_n283));
  nanp02aa1n03x5               g188(.a(new_n277), .b(new_n283), .o1(\s[28] ));
  norb02aa1n02x5               g189(.a(new_n275), .b(new_n276), .out0(new_n285));
  oaih12aa1n02x5               g190(.a(new_n285), .b(new_n280), .c(new_n281), .o1(new_n286));
  inv000aa1n03x5               g191(.a(new_n274), .o1(new_n287));
  oaoi03aa1n02x5               g192(.a(\a[28] ), .b(\b[27] ), .c(new_n287), .o1(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[28] ), .b(\a[29] ), .out0(new_n289));
  nona22aa1n03x5               g194(.a(new_n286), .b(new_n288), .c(new_n289), .out0(new_n290));
  aoai13aa1n02x7               g195(.a(new_n289), .b(new_n288), .c(new_n272), .d(new_n285), .o1(new_n291));
  nanp02aa1n03x5               g196(.a(new_n291), .b(new_n290), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n118), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g198(.a(new_n275), .b(new_n289), .c(new_n276), .out0(new_n294));
  oao003aa1n02x5               g199(.a(\a[28] ), .b(\b[27] ), .c(new_n287), .carry(new_n295));
  oaoi03aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .o1(new_n296));
  tech160nm_fixorc02aa1n03p5x5 g201(.a(\a[30] ), .b(\b[29] ), .out0(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  aoai13aa1n02x7               g203(.a(new_n298), .b(new_n296), .c(new_n272), .d(new_n294), .o1(new_n299));
  oaih12aa1n02x5               g204(.a(new_n294), .b(new_n280), .c(new_n281), .o1(new_n300));
  nona22aa1n03x5               g205(.a(new_n300), .b(new_n296), .c(new_n298), .out0(new_n301));
  nanp02aa1n03x5               g206(.a(new_n299), .b(new_n301), .o1(\s[30] ));
  nanp02aa1n02x5               g207(.a(new_n296), .b(new_n297), .o1(new_n303));
  oai012aa1n02x5               g208(.a(new_n303), .b(\b[29] ), .c(\a[30] ), .o1(new_n304));
  nano23aa1n02x4               g209(.a(new_n289), .b(new_n276), .c(new_n297), .d(new_n275), .out0(new_n305));
  oaih12aa1n02x5               g210(.a(new_n305), .b(new_n280), .c(new_n281), .o1(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  nona22aa1n03x5               g212(.a(new_n306), .b(new_n307), .c(new_n304), .out0(new_n308));
  aoai13aa1n02x7               g213(.a(new_n307), .b(new_n304), .c(new_n272), .d(new_n305), .o1(new_n309));
  nanp02aa1n03x5               g214(.a(new_n309), .b(new_n308), .o1(\s[31] ));
  xorb03aa1n02x5               g215(.a(new_n120), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi012aa1n02x5               g216(.a(new_n114), .b(new_n120), .c(new_n115), .o1(new_n312));
  xnrc02aa1n02x5               g217(.a(new_n312), .b(new_n113), .out0(\s[4] ));
  nanp02aa1n02x5               g218(.a(new_n121), .b(new_n122), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g220(.a(new_n107), .b(new_n314), .c(new_n123), .o1(new_n316));
  xnrb03aa1n02x5               g221(.a(new_n316), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g222(.a(new_n108), .b(new_n314), .c(new_n124), .o(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g224(.a(new_n102), .b(new_n318), .c(new_n103), .o1(new_n320));
  xnrb03aa1n02x5               g225(.a(new_n320), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g226(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


