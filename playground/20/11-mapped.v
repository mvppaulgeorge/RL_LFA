// Benchmark "adder" written by ABC on Wed Jul 17 22:16:42 2024

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
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n180, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n190, new_n191,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n305, new_n308, new_n310, new_n311, new_n312, new_n313,
    new_n314, new_n316;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n06x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  and002aa1n12x5               g003(.a(\b[1] ), .b(\a[2] ), .o(new_n99));
  nand22aa1n12x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nor042aa1n03x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oab012aa1n09x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .out0(new_n102));
  tech160nm_fixorc02aa1n04x5   g007(.a(\a[4] ), .b(\b[3] ), .out0(new_n103));
  nor042aa1d18x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1n10x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  norb02aa1n02x7               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nanp03aa1n06x5               g011(.a(new_n102), .b(new_n103), .c(new_n106), .o1(new_n107));
  inv030aa1n02x5               g012(.a(new_n104), .o1(new_n108));
  oao003aa1n06x5               g013(.a(\a[4] ), .b(\b[3] ), .c(new_n108), .carry(new_n109));
  nor042aa1n03x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand42aa1n20x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor002aa1d24x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand42aa1n16x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1n09x5               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  xorc02aa1n12x5               g019(.a(\a[6] ), .b(\b[5] ), .out0(new_n115));
  nor022aa1n16x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nand22aa1n09x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  norb02aa1n03x5               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  nanp03aa1n02x5               g023(.a(new_n114), .b(new_n115), .c(new_n118), .o1(new_n119));
  inv020aa1d32x5               g024(.a(\a[6] ), .o1(new_n120));
  inv040aa1d32x5               g025(.a(\b[5] ), .o1(new_n121));
  oao003aa1n03x5               g026(.a(new_n120), .b(new_n121), .c(new_n116), .carry(new_n122));
  inv000aa1n02x5               g027(.a(new_n112), .o1(new_n123));
  oaoi03aa1n03x5               g028(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n124));
  tech160nm_fiaoi012aa1n03p5x5 g029(.a(new_n124), .b(new_n114), .c(new_n122), .o1(new_n125));
  aoai13aa1n12x5               g030(.a(new_n125), .b(new_n119), .c(new_n107), .d(new_n109), .o1(new_n126));
  xorc02aa1n12x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoi012aa1n02x5               g032(.a(new_n98), .b(new_n126), .c(new_n127), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  tech160nm_fixnrc02aa1n02p5x5 g034(.a(\b[9] ), .b(\a[10] ), .out0(new_n130));
  norb02aa1n02x5               g035(.a(new_n127), .b(new_n130), .out0(new_n131));
  aob012aa1n02x5               g036(.a(new_n98), .b(\b[9] ), .c(\a[10] ), .out0(new_n132));
  oaib12aa1n09x5               g037(.a(new_n132), .b(\b[9] ), .c(new_n97), .out0(new_n133));
  ao0012aa1n03x7               g038(.a(new_n133), .b(new_n126), .c(new_n131), .o(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n04x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand42aa1d28x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  aoi012aa1n02x5               g042(.a(new_n136), .b(new_n134), .c(new_n137), .o1(new_n138));
  xnrb03aa1n03x5               g043(.a(new_n138), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor042aa1n04x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1n16x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nano23aa1d15x5               g046(.a(new_n136), .b(new_n140), .c(new_n141), .d(new_n137), .out0(new_n142));
  tech160nm_fiao0012aa1n05x5   g047(.a(new_n140), .b(new_n136), .c(new_n141), .o(new_n143));
  tech160nm_fiao0012aa1n02p5x5 g048(.a(new_n143), .b(new_n142), .c(new_n133), .o(new_n144));
  nanb03aa1d24x5               g049(.a(new_n130), .b(new_n142), .c(new_n127), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  xnrc02aa1n12x5               g051(.a(\b[12] ), .b(\a[13] ), .out0(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n144), .c(new_n126), .d(new_n146), .o1(new_n149));
  aoi112aa1n02x5               g054(.a(new_n144), .b(new_n148), .c(new_n126), .d(new_n146), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n149), .b(new_n150), .out0(\s[13] ));
  orn002aa1n02x5               g056(.a(\a[13] ), .b(\b[12] ), .o(new_n152));
  tech160nm_fixnrc02aa1n05x5   g057(.a(\b[13] ), .b(\a[14] ), .out0(new_n153));
  xobna2aa1n03x5               g058(.a(new_n153), .b(new_n149), .c(new_n152), .out0(\s[14] ));
  nona32aa1n09x5               g059(.a(new_n126), .b(new_n153), .c(new_n147), .d(new_n145), .out0(new_n155));
  nor042aa1n06x5               g060(.a(new_n153), .b(new_n147), .o1(new_n156));
  aoai13aa1n06x5               g061(.a(new_n156), .b(new_n143), .c(new_n142), .d(new_n133), .o1(new_n157));
  oao003aa1n02x5               g062(.a(\a[14] ), .b(\b[13] ), .c(new_n152), .carry(new_n158));
  nand02aa1d06x5               g063(.a(new_n157), .b(new_n158), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  xnrc02aa1n12x5               g065(.a(\b[14] ), .b(\a[15] ), .out0(new_n161));
  xobna2aa1n03x5               g066(.a(new_n161), .b(new_n155), .c(new_n160), .out0(\s[15] ));
  nor002aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  ao0012aa1n03x7               g068(.a(new_n161), .b(new_n155), .c(new_n160), .o(new_n164));
  xnrc02aa1n12x5               g069(.a(\b[15] ), .b(\a[16] ), .out0(new_n165));
  oaib12aa1n03x5               g070(.a(new_n165), .b(new_n163), .c(new_n164), .out0(new_n166));
  nona22aa1n02x4               g071(.a(new_n164), .b(new_n165), .c(new_n163), .out0(new_n167));
  nanp02aa1n03x5               g072(.a(new_n166), .b(new_n167), .o1(\s[16] ));
  inv000aa1d42x5               g073(.a(\a[17] ), .o1(new_n169));
  nor042aa1n06x5               g074(.a(new_n165), .b(new_n161), .o1(new_n170));
  nano22aa1n06x5               g075(.a(new_n145), .b(new_n156), .c(new_n170), .out0(new_n171));
  inv000aa1n02x5               g076(.a(new_n170), .o1(new_n172));
  tech160nm_fiaoi012aa1n04x5   g077(.a(new_n172), .b(new_n157), .c(new_n158), .o1(new_n173));
  inv000aa1d42x5               g078(.a(\a[16] ), .o1(new_n174));
  inv000aa1d42x5               g079(.a(\b[15] ), .o1(new_n175));
  oaoi03aa1n02x5               g080(.a(new_n174), .b(new_n175), .c(new_n163), .o1(new_n176));
  inv000aa1n02x5               g081(.a(new_n176), .o1(new_n177));
  aoi112aa1n09x5               g082(.a(new_n173), .b(new_n177), .c(new_n126), .d(new_n171), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[16] ), .c(new_n169), .out0(\s[17] ));
  oaoi03aa1n03x5               g084(.a(\a[17] ), .b(\b[16] ), .c(new_n178), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g086(.a(\a[18] ), .o1(new_n182));
  xroi22aa1d06x4               g087(.a(new_n169), .b(\b[16] ), .c(new_n182), .d(\b[17] ), .out0(new_n183));
  inv000aa1n02x5               g088(.a(new_n183), .o1(new_n184));
  aoi112aa1n06x5               g089(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n185));
  aoib12aa1n09x5               g090(.a(new_n185), .b(new_n182), .c(\b[17] ), .out0(new_n186));
  tech160nm_fioai012aa1n05x5   g091(.a(new_n186), .b(new_n178), .c(new_n184), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g093(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d24x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  nand02aa1n06x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n191), .b(new_n190), .out0(new_n192));
  nor002aa1d32x5               g097(.a(\b[19] ), .b(\a[20] ), .o1(new_n193));
  nand22aa1n12x5               g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  nanb02aa1n02x5               g099(.a(new_n193), .b(new_n194), .out0(new_n195));
  aoai13aa1n03x5               g100(.a(new_n195), .b(new_n190), .c(new_n187), .d(new_n192), .o1(new_n196));
  nand02aa1n03x5               g101(.a(new_n126), .b(new_n171), .o1(new_n197));
  nand22aa1n03x5               g102(.a(new_n159), .b(new_n170), .o1(new_n198));
  nand23aa1n06x5               g103(.a(new_n197), .b(new_n198), .c(new_n176), .o1(new_n199));
  nor042aa1n03x5               g104(.a(\b[16] ), .b(\a[17] ), .o1(new_n200));
  aob012aa1n02x5               g105(.a(new_n200), .b(\b[17] ), .c(\a[18] ), .out0(new_n201));
  oaib12aa1n09x5               g106(.a(new_n201), .b(\b[17] ), .c(new_n182), .out0(new_n202));
  aoai13aa1n03x5               g107(.a(new_n192), .b(new_n202), .c(new_n199), .d(new_n183), .o1(new_n203));
  nona22aa1n03x5               g108(.a(new_n203), .b(new_n195), .c(new_n190), .out0(new_n204));
  nanp02aa1n03x5               g109(.a(new_n196), .b(new_n204), .o1(\s[20] ));
  nona23aa1d16x5               g110(.a(new_n194), .b(new_n191), .c(new_n190), .d(new_n193), .out0(new_n206));
  ao0012aa1n12x5               g111(.a(new_n193), .b(new_n190), .c(new_n194), .o(new_n207));
  oabi12aa1n18x5               g112(.a(new_n207), .b(new_n206), .c(new_n186), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  norb02aa1n03x5               g114(.a(new_n183), .b(new_n206), .out0(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  tech160nm_fioai012aa1n05x5   g116(.a(new_n209), .b(new_n178), .c(new_n211), .o1(new_n212));
  xorb03aa1n02x5               g117(.a(new_n212), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  xnrc02aa1n12x5               g119(.a(\b[20] ), .b(\a[21] ), .out0(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  xnrc02aa1n12x5               g121(.a(\b[21] ), .b(\a[22] ), .out0(new_n217));
  aoai13aa1n03x5               g122(.a(new_n217), .b(new_n214), .c(new_n212), .d(new_n216), .o1(new_n218));
  aoai13aa1n03x5               g123(.a(new_n216), .b(new_n208), .c(new_n199), .d(new_n210), .o1(new_n219));
  nona22aa1n03x5               g124(.a(new_n219), .b(new_n217), .c(new_n214), .out0(new_n220));
  nanp02aa1n03x5               g125(.a(new_n218), .b(new_n220), .o1(\s[22] ));
  nano23aa1n03x7               g126(.a(new_n190), .b(new_n193), .c(new_n194), .d(new_n191), .out0(new_n222));
  nor042aa1n09x5               g127(.a(new_n217), .b(new_n215), .o1(new_n223));
  nano22aa1n12x5               g128(.a(new_n184), .b(new_n223), .c(new_n222), .out0(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\a[22] ), .o1(new_n226));
  inv000aa1d42x5               g131(.a(\b[21] ), .o1(new_n227));
  oaoi03aa1n12x5               g132(.a(new_n226), .b(new_n227), .c(new_n214), .o1(new_n228));
  inv000aa1n02x5               g133(.a(new_n228), .o1(new_n229));
  aoi012aa1d24x5               g134(.a(new_n229), .b(new_n208), .c(new_n223), .o1(new_n230));
  tech160nm_fioai012aa1n05x5   g135(.a(new_n230), .b(new_n178), .c(new_n225), .o1(new_n231));
  xorb03aa1n02x5               g136(.a(new_n231), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n12x5               g138(.a(\a[23] ), .b(\b[22] ), .out0(new_n234));
  tech160nm_fixnrc02aa1n05x5   g139(.a(\b[23] ), .b(\a[24] ), .out0(new_n235));
  aoai13aa1n03x5               g140(.a(new_n235), .b(new_n233), .c(new_n231), .d(new_n234), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n230), .o1(new_n237));
  aoai13aa1n03x5               g142(.a(new_n234), .b(new_n237), .c(new_n199), .d(new_n224), .o1(new_n238));
  nona22aa1n03x5               g143(.a(new_n238), .b(new_n235), .c(new_n233), .out0(new_n239));
  nanp02aa1n03x5               g144(.a(new_n236), .b(new_n239), .o1(\s[24] ));
  norb02aa1n03x4               g145(.a(new_n234), .b(new_n235), .out0(new_n241));
  inv020aa1n02x5               g146(.a(new_n241), .o1(new_n242));
  nano32aa1n02x5               g147(.a(new_n242), .b(new_n183), .c(new_n223), .d(new_n222), .out0(new_n243));
  inv000aa1n02x5               g148(.a(new_n243), .o1(new_n244));
  aoai13aa1n06x5               g149(.a(new_n223), .b(new_n207), .c(new_n222), .d(new_n202), .o1(new_n245));
  aoi112aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n246));
  oab012aa1n02x4               g151(.a(new_n246), .b(\a[24] ), .c(\b[23] ), .out0(new_n247));
  aoai13aa1n02x7               g152(.a(new_n247), .b(new_n242), .c(new_n245), .d(new_n228), .o1(new_n248));
  inv000aa1n02x5               g153(.a(new_n248), .o1(new_n249));
  tech160nm_fioai012aa1n05x5   g154(.a(new_n249), .b(new_n178), .c(new_n244), .o1(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor002aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  xorc02aa1n06x5               g157(.a(\a[25] ), .b(\b[24] ), .out0(new_n253));
  xnrc02aa1n12x5               g158(.a(\b[25] ), .b(\a[26] ), .out0(new_n254));
  aoai13aa1n03x5               g159(.a(new_n254), .b(new_n252), .c(new_n250), .d(new_n253), .o1(new_n255));
  aoai13aa1n03x5               g160(.a(new_n253), .b(new_n248), .c(new_n199), .d(new_n243), .o1(new_n256));
  nona22aa1n03x5               g161(.a(new_n256), .b(new_n254), .c(new_n252), .out0(new_n257));
  nanp02aa1n03x5               g162(.a(new_n255), .b(new_n257), .o1(\s[26] ));
  norb02aa1n02x5               g163(.a(new_n253), .b(new_n254), .out0(new_n259));
  inv000aa1n02x5               g164(.a(new_n259), .o1(new_n260));
  nona22aa1d24x5               g165(.a(new_n224), .b(new_n260), .c(new_n242), .out0(new_n261));
  inv000aa1d42x5               g166(.a(\a[26] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(\b[25] ), .o1(new_n263));
  tech160nm_fioaoi03aa1n03p5x5 g168(.a(new_n262), .b(new_n263), .c(new_n252), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  tech160nm_fiaoi012aa1n05x5   g170(.a(new_n265), .b(new_n248), .c(new_n259), .o1(new_n266));
  oai012aa1n09x5               g171(.a(new_n266), .b(new_n178), .c(new_n261), .o1(new_n267));
  xorb03aa1n02x5               g172(.a(new_n267), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  xorc02aa1n02x5               g174(.a(\a[27] ), .b(\b[26] ), .out0(new_n270));
  xnrc02aa1n02x5               g175(.a(\b[27] ), .b(\a[28] ), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n269), .c(new_n267), .d(new_n270), .o1(new_n272));
  inv040aa1n03x5               g177(.a(new_n261), .o1(new_n273));
  aoai13aa1n06x5               g178(.a(new_n241), .b(new_n229), .c(new_n208), .d(new_n223), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n264), .b(new_n260), .c(new_n274), .d(new_n247), .o1(new_n275));
  aoai13aa1n02x5               g180(.a(new_n270), .b(new_n275), .c(new_n199), .d(new_n273), .o1(new_n276));
  nona22aa1n02x4               g181(.a(new_n276), .b(new_n271), .c(new_n269), .out0(new_n277));
  nanp02aa1n03x5               g182(.a(new_n272), .b(new_n277), .o1(\s[28] ));
  norb02aa1n02x5               g183(.a(new_n270), .b(new_n271), .out0(new_n279));
  aoai13aa1n02x5               g184(.a(new_n279), .b(new_n275), .c(new_n199), .d(new_n273), .o1(new_n280));
  aob012aa1n03x5               g185(.a(new_n269), .b(\b[27] ), .c(\a[28] ), .out0(new_n281));
  oa0012aa1n12x5               g186(.a(new_n281), .b(\b[27] ), .c(\a[28] ), .o(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  nona22aa1n02x4               g189(.a(new_n280), .b(new_n283), .c(new_n284), .out0(new_n285));
  aoai13aa1n03x5               g190(.a(new_n284), .b(new_n283), .c(new_n267), .d(new_n279), .o1(new_n286));
  nanp02aa1n03x5               g191(.a(new_n286), .b(new_n285), .o1(\s[29] ));
  xorb03aa1n02x5               g192(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g193(.a(new_n270), .b(new_n284), .c(new_n271), .out0(new_n289));
  oaoi03aa1n09x5               g194(.a(\a[29] ), .b(\b[28] ), .c(new_n282), .o1(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[29] ), .b(\a[30] ), .out0(new_n291));
  aoai13aa1n03x5               g196(.a(new_n291), .b(new_n290), .c(new_n267), .d(new_n289), .o1(new_n292));
  aoai13aa1n02x5               g197(.a(new_n289), .b(new_n275), .c(new_n199), .d(new_n273), .o1(new_n293));
  nona22aa1n02x4               g198(.a(new_n293), .b(new_n290), .c(new_n291), .out0(new_n294));
  nanp02aa1n03x5               g199(.a(new_n292), .b(new_n294), .o1(\s[30] ));
  norb02aa1n02x5               g200(.a(new_n289), .b(new_n291), .out0(new_n296));
  aoai13aa1n02x5               g201(.a(new_n296), .b(new_n275), .c(new_n199), .d(new_n273), .o1(new_n297));
  nanb02aa1n02x5               g202(.a(new_n291), .b(new_n290), .out0(new_n298));
  oai012aa1n03x5               g203(.a(new_n298), .b(\b[29] ), .c(\a[30] ), .o1(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[30] ), .b(\a[31] ), .out0(new_n300));
  nona22aa1n02x4               g205(.a(new_n297), .b(new_n299), .c(new_n300), .out0(new_n301));
  aoai13aa1n03x5               g206(.a(new_n300), .b(new_n299), .c(new_n267), .d(new_n296), .o1(new_n302));
  nanp02aa1n03x5               g207(.a(new_n302), .b(new_n301), .o1(\s[31] ));
  xobna2aa1n03x5               g208(.a(new_n102), .b(new_n105), .c(new_n108), .out0(\s[3] ));
  oai012aa1n02x5               g209(.a(new_n105), .b(new_n102), .c(new_n104), .o1(new_n305));
  xnrb03aa1n02x5               g210(.a(new_n305), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnbna2aa1n03x5               g211(.a(new_n118), .b(new_n107), .c(new_n109), .out0(\s[5] ));
  nanp03aa1n02x5               g212(.a(new_n107), .b(new_n109), .c(new_n118), .o1(new_n308));
  xobna2aa1n03x5               g213(.a(new_n115), .b(new_n308), .c(new_n117), .out0(\s[6] ));
  norb02aa1n02x5               g214(.a(new_n113), .b(new_n112), .out0(new_n310));
  nanp02aa1n02x5               g215(.a(new_n308), .b(new_n117), .o1(new_n311));
  nanp02aa1n02x5               g216(.a(new_n311), .b(new_n115), .o1(new_n312));
  oai112aa1n02x5               g217(.a(new_n312), .b(new_n310), .c(new_n121), .d(new_n120), .o1(new_n313));
  oaoi13aa1n02x5               g218(.a(new_n310), .b(new_n312), .c(new_n120), .d(new_n121), .o1(new_n314));
  norb02aa1n02x5               g219(.a(new_n313), .b(new_n314), .out0(\s[7] ));
  norb02aa1n02x5               g220(.a(new_n111), .b(new_n110), .out0(new_n316));
  xnbna2aa1n03x5               g221(.a(new_n316), .b(new_n313), .c(new_n123), .out0(\s[8] ));
  xorb03aa1n02x5               g222(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


