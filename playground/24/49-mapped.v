// Benchmark "adder" written by ABC on Thu Jul 18 00:43:20 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n230, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n323, new_n325, new_n327;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nor002aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  tech160nm_fioai012aa1n04x5   g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  nor002aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand42aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n06x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n03x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  tech160nm_fiao0012aa1n02p5x5 g012(.a(new_n103), .b(new_n105), .c(new_n104), .o(new_n108));
  oabi12aa1n06x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .out0(new_n109));
  nor042aa1n06x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nand02aa1d20x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  norb02aa1n06x5               g016(.a(new_n111), .b(new_n110), .out0(new_n112));
  nor002aa1n06x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nand42aa1n06x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  norb02aa1n02x7               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  tech160nm_fixnrc02aa1n02p5x5 g020(.a(\b[6] ), .b(\a[7] ), .out0(new_n116));
  xorc02aa1n02x5               g021(.a(\a[5] ), .b(\b[4] ), .out0(new_n117));
  nano32aa1n03x7               g022(.a(new_n116), .b(new_n117), .c(new_n112), .d(new_n115), .out0(new_n118));
  nanp02aa1n06x5               g023(.a(new_n118), .b(new_n109), .o1(new_n119));
  aoi112aa1n02x7               g024(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[5] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[4] ), .o1(new_n122));
  tech160nm_fiaoi012aa1n04x5   g027(.a(new_n110), .b(new_n121), .c(new_n122), .o1(new_n123));
  inv020aa1n02x5               g028(.a(new_n123), .o1(new_n124));
  inv030aa1n02x5               g029(.a(new_n113), .o1(new_n125));
  nano32aa1n02x5               g030(.a(new_n116), .b(new_n125), .c(new_n114), .d(new_n111), .out0(new_n126));
  aoi112aa1n06x5               g031(.a(new_n113), .b(new_n120), .c(new_n126), .d(new_n124), .o1(new_n127));
  nand42aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nanb02aa1n03x5               g033(.a(new_n97), .b(new_n128), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n98), .b(new_n129), .c(new_n119), .d(new_n127), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1n06x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nanp02aa1n04x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nona23aa1n02x4               g038(.a(new_n128), .b(new_n133), .c(new_n132), .d(new_n97), .out0(new_n134));
  oai012aa1n12x5               g039(.a(new_n133), .b(new_n97), .c(new_n132), .o1(new_n135));
  aoai13aa1n02x5               g040(.a(new_n135), .b(new_n134), .c(new_n119), .d(new_n127), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n06x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nanp02aa1n09x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  aoi012aa1n02x5               g044(.a(new_n138), .b(new_n136), .c(new_n139), .o1(new_n140));
  xnrb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nanb02aa1n02x5               g046(.a(new_n132), .b(new_n133), .out0(new_n142));
  nor022aa1n08x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand22aa1n12x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nano23aa1n09x5               g049(.a(new_n138), .b(new_n143), .c(new_n144), .d(new_n139), .out0(new_n145));
  nona22aa1n09x5               g050(.a(new_n145), .b(new_n142), .c(new_n129), .out0(new_n146));
  nona23aa1d18x5               g051(.a(new_n144), .b(new_n139), .c(new_n138), .d(new_n143), .out0(new_n147));
  ao0012aa1n12x5               g052(.a(new_n143), .b(new_n138), .c(new_n144), .o(new_n148));
  oabi12aa1n18x5               g053(.a(new_n148), .b(new_n147), .c(new_n135), .out0(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  aoai13aa1n06x5               g055(.a(new_n150), .b(new_n146), .c(new_n119), .d(new_n127), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g057(.a(\a[14] ), .o1(new_n153));
  norp02aa1n04x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  tech160nm_fixnrc02aa1n05x5   g059(.a(\b[12] ), .b(\a[13] ), .out0(new_n155));
  aoib12aa1n02x5               g060(.a(new_n154), .b(new_n151), .c(new_n155), .out0(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(new_n153), .out0(\s[14] ));
  nor042aa1n03x5               g062(.a(\b[14] ), .b(\a[15] ), .o1(new_n158));
  nand42aa1n06x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  nanb02aa1n02x5               g064(.a(new_n158), .b(new_n159), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  inv000aa1d42x5               g066(.a(\b[13] ), .o1(new_n162));
  oaoi03aa1n09x5               g067(.a(new_n153), .b(new_n162), .c(new_n154), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  xnrc02aa1n02x5               g069(.a(\b[13] ), .b(\a[14] ), .out0(new_n165));
  nor042aa1n04x5               g070(.a(new_n165), .b(new_n155), .o1(new_n166));
  aoai13aa1n06x5               g071(.a(new_n161), .b(new_n164), .c(new_n151), .d(new_n166), .o1(new_n167));
  aoi112aa1n02x5               g072(.a(new_n164), .b(new_n161), .c(new_n151), .d(new_n166), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(\s[15] ));
  nor042aa1n03x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand42aa1n04x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  oai112aa1n03x5               g077(.a(new_n167), .b(new_n172), .c(\b[14] ), .d(\a[15] ), .o1(new_n173));
  oaoi13aa1n06x5               g078(.a(new_n172), .b(new_n167), .c(\a[15] ), .d(\b[14] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n173), .b(new_n174), .out0(\s[16] ));
  inv000aa1d42x5               g080(.a(new_n111), .o1(new_n176));
  xorc02aa1n02x5               g081(.a(\a[7] ), .b(\b[6] ), .out0(new_n177));
  nona23aa1n02x4               g082(.a(new_n177), .b(new_n115), .c(new_n123), .d(new_n176), .out0(new_n178));
  nona22aa1n02x4               g083(.a(new_n178), .b(new_n120), .c(new_n113), .out0(new_n179));
  nano23aa1d15x5               g084(.a(new_n158), .b(new_n170), .c(new_n171), .d(new_n159), .out0(new_n180));
  nano22aa1d15x5               g085(.a(new_n146), .b(new_n166), .c(new_n180), .out0(new_n181));
  aoai13aa1n06x5               g086(.a(new_n181), .b(new_n179), .c(new_n118), .d(new_n109), .o1(new_n182));
  aoai13aa1n12x5               g087(.a(new_n180), .b(new_n164), .c(new_n149), .d(new_n166), .o1(new_n183));
  aoi012aa1n02x7               g088(.a(new_n170), .b(new_n158), .c(new_n171), .o1(new_n184));
  nand23aa1n06x5               g089(.a(new_n182), .b(new_n183), .c(new_n184), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g091(.a(\a[18] ), .o1(new_n187));
  inv040aa1d32x5               g092(.a(\a[17] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[16] ), .o1(new_n189));
  oaoi03aa1n03x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  nand02aa1d06x5               g096(.a(new_n119), .b(new_n127), .o1(new_n192));
  inv000aa1d42x5               g097(.a(new_n180), .o1(new_n193));
  inv040aa1n02x5               g098(.a(new_n135), .o1(new_n194));
  aoai13aa1n06x5               g099(.a(new_n166), .b(new_n148), .c(new_n145), .d(new_n194), .o1(new_n195));
  aoai13aa1n06x5               g100(.a(new_n184), .b(new_n193), .c(new_n195), .d(new_n163), .o1(new_n196));
  xroi22aa1d06x4               g101(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n197));
  aoai13aa1n06x5               g102(.a(new_n197), .b(new_n196), .c(new_n192), .d(new_n181), .o1(new_n198));
  oai022aa1n04x7               g103(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n199));
  oaib12aa1n09x5               g104(.a(new_n199), .b(new_n187), .c(\b[17] ), .out0(new_n200));
  nor002aa1d32x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nanp02aa1n12x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  xnbna2aa1n03x5               g109(.a(new_n204), .b(new_n198), .c(new_n200), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g111(.a(new_n201), .o1(new_n207));
  aoi012aa1n03x5               g112(.a(new_n203), .b(new_n198), .c(new_n200), .o1(new_n208));
  nor022aa1n16x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nand02aa1d28x5               g114(.a(\b[19] ), .b(\a[20] ), .o1(new_n210));
  nanb02aa1n02x5               g115(.a(new_n209), .b(new_n210), .out0(new_n211));
  nano22aa1n03x5               g116(.a(new_n208), .b(new_n207), .c(new_n211), .out0(new_n212));
  nanp02aa1n02x5               g117(.a(new_n189), .b(new_n188), .o1(new_n213));
  oaoi03aa1n03x5               g118(.a(\a[18] ), .b(\b[17] ), .c(new_n213), .o1(new_n214));
  aoai13aa1n03x5               g119(.a(new_n204), .b(new_n214), .c(new_n185), .d(new_n197), .o1(new_n215));
  aoi012aa1n02x7               g120(.a(new_n211), .b(new_n215), .c(new_n207), .o1(new_n216));
  nor002aa1n02x5               g121(.a(new_n216), .b(new_n212), .o1(\s[20] ));
  nano23aa1n09x5               g122(.a(new_n201), .b(new_n209), .c(new_n210), .d(new_n202), .out0(new_n218));
  nand02aa1n04x5               g123(.a(new_n197), .b(new_n218), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  aoai13aa1n06x5               g125(.a(new_n220), .b(new_n196), .c(new_n192), .d(new_n181), .o1(new_n221));
  nona23aa1n09x5               g126(.a(new_n210), .b(new_n202), .c(new_n201), .d(new_n209), .out0(new_n222));
  aoi012aa1n12x5               g127(.a(new_n209), .b(new_n201), .c(new_n210), .o1(new_n223));
  oai012aa1n18x5               g128(.a(new_n223), .b(new_n222), .c(new_n200), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  nor002aa1d32x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  nanp02aa1n04x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n227), .b(new_n226), .out0(new_n228));
  xnbna2aa1n03x5               g133(.a(new_n228), .b(new_n221), .c(new_n225), .out0(\s[21] ));
  inv000aa1d42x5               g134(.a(new_n226), .o1(new_n230));
  aobi12aa1n02x5               g135(.a(new_n228), .b(new_n221), .c(new_n225), .out0(new_n231));
  xnrc02aa1n12x5               g136(.a(\b[21] ), .b(\a[22] ), .out0(new_n232));
  nano22aa1n02x4               g137(.a(new_n231), .b(new_n230), .c(new_n232), .out0(new_n233));
  aoai13aa1n03x5               g138(.a(new_n228), .b(new_n224), .c(new_n185), .d(new_n220), .o1(new_n234));
  tech160nm_fiaoi012aa1n02p5x5 g139(.a(new_n232), .b(new_n234), .c(new_n230), .o1(new_n235));
  nor002aa1n02x5               g140(.a(new_n235), .b(new_n233), .o1(\s[22] ));
  nano22aa1n12x5               g141(.a(new_n232), .b(new_n230), .c(new_n227), .out0(new_n237));
  and003aa1n02x5               g142(.a(new_n197), .b(new_n237), .c(new_n218), .o(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n196), .c(new_n192), .d(new_n181), .o1(new_n239));
  oao003aa1n12x5               g144(.a(\a[22] ), .b(\b[21] ), .c(new_n230), .carry(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  aoi012aa1d18x5               g146(.a(new_n241), .b(new_n224), .c(new_n237), .o1(new_n242));
  xnrc02aa1n12x5               g147(.a(\b[22] ), .b(\a[23] ), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  xnbna2aa1n03x5               g149(.a(new_n244), .b(new_n239), .c(new_n242), .out0(\s[23] ));
  nor042aa1n06x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  tech160nm_fiaoi012aa1n02p5x5 g152(.a(new_n243), .b(new_n239), .c(new_n242), .o1(new_n248));
  tech160nm_fixnrc02aa1n02p5x5 g153(.a(\b[23] ), .b(\a[24] ), .out0(new_n249));
  nano22aa1n02x4               g154(.a(new_n248), .b(new_n247), .c(new_n249), .out0(new_n250));
  inv000aa1d42x5               g155(.a(new_n242), .o1(new_n251));
  aoai13aa1n03x5               g156(.a(new_n244), .b(new_n251), .c(new_n185), .d(new_n238), .o1(new_n252));
  aoi012aa1n03x5               g157(.a(new_n249), .b(new_n252), .c(new_n247), .o1(new_n253));
  norp02aa1n03x5               g158(.a(new_n253), .b(new_n250), .o1(\s[24] ));
  nor042aa1n06x5               g159(.a(new_n249), .b(new_n243), .o1(new_n255));
  nano22aa1n06x5               g160(.a(new_n219), .b(new_n237), .c(new_n255), .out0(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n196), .c(new_n192), .d(new_n181), .o1(new_n257));
  inv020aa1n03x5               g162(.a(new_n223), .o1(new_n258));
  aoai13aa1n06x5               g163(.a(new_n237), .b(new_n258), .c(new_n218), .d(new_n214), .o1(new_n259));
  inv030aa1n02x5               g164(.a(new_n255), .o1(new_n260));
  oao003aa1n02x5               g165(.a(\a[24] ), .b(\b[23] ), .c(new_n247), .carry(new_n261));
  aoai13aa1n12x5               g166(.a(new_n261), .b(new_n260), .c(new_n259), .d(new_n240), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  xnrc02aa1n12x5               g168(.a(\b[24] ), .b(\a[25] ), .out0(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  xnbna2aa1n03x5               g170(.a(new_n265), .b(new_n257), .c(new_n263), .out0(\s[25] ));
  nor042aa1n03x5               g171(.a(\b[24] ), .b(\a[25] ), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  tech160nm_fiaoi012aa1n02p5x5 g173(.a(new_n264), .b(new_n257), .c(new_n263), .o1(new_n269));
  xnrc02aa1n02x5               g174(.a(\b[25] ), .b(\a[26] ), .out0(new_n270));
  nano22aa1n02x4               g175(.a(new_n269), .b(new_n268), .c(new_n270), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n265), .b(new_n262), .c(new_n185), .d(new_n256), .o1(new_n272));
  aoi012aa1n03x5               g177(.a(new_n270), .b(new_n272), .c(new_n268), .o1(new_n273));
  nor002aa1n02x5               g178(.a(new_n273), .b(new_n271), .o1(\s[26] ));
  nor042aa1n06x5               g179(.a(new_n270), .b(new_n264), .o1(new_n275));
  nano32aa1d12x5               g180(.a(new_n219), .b(new_n275), .c(new_n237), .d(new_n255), .out0(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n196), .c(new_n192), .d(new_n181), .o1(new_n277));
  oao003aa1n02x5               g182(.a(\a[26] ), .b(\b[25] ), .c(new_n268), .carry(new_n278));
  aobi12aa1n12x5               g183(.a(new_n278), .b(new_n262), .c(new_n275), .out0(new_n279));
  norp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  nanp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  norb02aa1n02x5               g186(.a(new_n281), .b(new_n280), .out0(new_n282));
  xnbna2aa1n03x5               g187(.a(new_n282), .b(new_n277), .c(new_n279), .out0(\s[27] ));
  inv000aa1n06x5               g188(.a(new_n280), .o1(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[27] ), .b(\a[28] ), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n255), .b(new_n241), .c(new_n224), .d(new_n237), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n275), .o1(new_n287));
  aoai13aa1n04x5               g192(.a(new_n278), .b(new_n287), .c(new_n286), .d(new_n261), .o1(new_n288));
  aoai13aa1n02x7               g193(.a(new_n281), .b(new_n288), .c(new_n185), .d(new_n276), .o1(new_n289));
  tech160nm_fiaoi012aa1n02p5x5 g194(.a(new_n285), .b(new_n289), .c(new_n284), .o1(new_n290));
  aobi12aa1n03x5               g195(.a(new_n281), .b(new_n277), .c(new_n279), .out0(new_n291));
  nano22aa1n03x5               g196(.a(new_n291), .b(new_n284), .c(new_n285), .out0(new_n292));
  norp02aa1n03x5               g197(.a(new_n290), .b(new_n292), .o1(\s[28] ));
  nano22aa1n02x4               g198(.a(new_n285), .b(new_n284), .c(new_n281), .out0(new_n294));
  aoai13aa1n02x5               g199(.a(new_n294), .b(new_n288), .c(new_n185), .d(new_n276), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[28] ), .b(\a[29] ), .out0(new_n297));
  tech160nm_fiaoi012aa1n02p5x5 g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n03x5               g203(.a(new_n294), .b(new_n277), .c(new_n279), .out0(new_n299));
  nano22aa1n03x5               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n03x5               g205(.a(new_n298), .b(new_n300), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g207(.a(new_n282), .b(new_n297), .c(new_n285), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n288), .c(new_n185), .d(new_n276), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[29] ), .b(\a[30] ), .out0(new_n306));
  tech160nm_fiaoi012aa1n02p5x5 g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  aobi12aa1n03x5               g212(.a(new_n303), .b(new_n277), .c(new_n279), .out0(new_n308));
  nano22aa1n03x5               g213(.a(new_n308), .b(new_n305), .c(new_n306), .out0(new_n309));
  norp02aa1n03x5               g214(.a(new_n307), .b(new_n309), .o1(\s[30] ));
  xnrc02aa1n02x5               g215(.a(\b[30] ), .b(\a[31] ), .out0(new_n311));
  norb03aa1n02x5               g216(.a(new_n294), .b(new_n306), .c(new_n297), .out0(new_n312));
  aobi12aa1n03x5               g217(.a(new_n312), .b(new_n277), .c(new_n279), .out0(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n314));
  nano22aa1n03x5               g219(.a(new_n313), .b(new_n311), .c(new_n314), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n312), .b(new_n288), .c(new_n185), .d(new_n276), .o1(new_n316));
  tech160nm_fiaoi012aa1n02p5x5 g221(.a(new_n311), .b(new_n316), .c(new_n314), .o1(new_n317));
  norp02aa1n03x5               g222(.a(new_n317), .b(new_n315), .o1(\s[31] ));
  xnrb03aa1n02x5               g223(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g224(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g226(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g227(.a(new_n121), .b(new_n122), .c(new_n109), .o1(new_n323));
  xnrc02aa1n02x5               g228(.a(new_n323), .b(new_n112), .out0(\s[6] ));
  oab012aa1n06x5               g229(.a(new_n110), .b(new_n323), .c(new_n176), .out0(new_n325));
  xnrc02aa1n02x5               g230(.a(new_n325), .b(new_n177), .out0(\s[7] ));
  oaoi03aa1n02x5               g231(.a(\a[7] ), .b(\b[6] ), .c(new_n325), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xobna2aa1n03x5               g233(.a(new_n129), .b(new_n119), .c(new_n127), .out0(\s[9] ));
endmodule

