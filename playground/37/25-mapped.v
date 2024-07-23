// Benchmark "adder" written by ABC on Thu Jul 18 07:08:03 2024

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
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n220, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n310,
    new_n313, new_n314, new_n316, new_n317, new_n319;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor002aa1d32x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  nor002aa1n20x5               g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  nand42aa1n04x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  nanb02aa1n06x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  xnrc02aa1n12x5               g009(.a(\b[6] ), .b(\a[7] ), .out0(new_n105));
  nor002aa1d32x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  nand22aa1n12x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nor042aa1n04x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  aoi012aa1n12x5               g013(.a(new_n106), .b(new_n108), .c(new_n107), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  tech160nm_fiaoi012aa1n05x5   g015(.a(new_n102), .b(new_n110), .c(new_n103), .o1(new_n111));
  oai013aa1d12x5               g016(.a(new_n111), .b(new_n105), .c(new_n109), .d(new_n104), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  nand22aa1n04x5               g018(.a(\b[0] ), .b(\a[1] ), .o1(new_n114));
  nor042aa1n02x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  oai012aa1n04x7               g020(.a(new_n113), .b(new_n115), .c(new_n114), .o1(new_n116));
  nand42aa1n03x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nor022aa1n16x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nor022aa1n06x5               g023(.a(\b[3] ), .b(\a[4] ), .o1(new_n119));
  tech160nm_finand02aa1n03p5x5 g024(.a(\b[3] ), .b(\a[4] ), .o1(new_n120));
  nona23aa1n09x5               g025(.a(new_n117), .b(new_n120), .c(new_n119), .d(new_n118), .out0(new_n121));
  tech160nm_fioai012aa1n03p5x5 g026(.a(new_n120), .b(new_n119), .c(new_n118), .o1(new_n122));
  oai012aa1n12x5               g027(.a(new_n122), .b(new_n121), .c(new_n116), .o1(new_n123));
  tech160nm_fixnrc02aa1n02p5x5 g028(.a(\b[4] ), .b(\a[5] ), .out0(new_n124));
  nona23aa1n03x5               g029(.a(new_n107), .b(new_n103), .c(new_n102), .d(new_n106), .out0(new_n125));
  nor043aa1n06x5               g030(.a(new_n125), .b(new_n124), .c(new_n105), .o1(new_n126));
  xorc02aa1n12x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  aoai13aa1n06x5               g032(.a(new_n127), .b(new_n112), .c(new_n123), .d(new_n126), .o1(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n99), .b(new_n128), .c(new_n101), .out0(\s[10] ));
  inv000aa1d42x5               g034(.a(new_n97), .o1(new_n130));
  inv000aa1n02x5               g035(.a(new_n98), .o1(new_n131));
  aoai13aa1n06x5               g036(.a(new_n130), .b(new_n131), .c(new_n128), .d(new_n101), .o1(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1n20x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1n08x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nor002aa1n16x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nand42aa1n16x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n138), .b(new_n134), .c(new_n132), .d(new_n135), .o1(new_n139));
  aoi112aa1n02x5               g044(.a(new_n134), .b(new_n138), .c(new_n132), .d(new_n135), .o1(new_n140));
  norb02aa1n03x4               g045(.a(new_n139), .b(new_n140), .out0(\s[12] ));
  nona32aa1n03x5               g046(.a(new_n127), .b(new_n131), .c(new_n134), .d(new_n97), .out0(new_n142));
  nanb03aa1n12x5               g047(.a(new_n136), .b(new_n137), .c(new_n135), .out0(new_n143));
  norp02aa1n02x5               g048(.a(new_n142), .b(new_n143), .o1(new_n144));
  aoai13aa1n06x5               g049(.a(new_n144), .b(new_n112), .c(new_n123), .d(new_n126), .o1(new_n145));
  aoi012aa1n12x5               g050(.a(new_n97), .b(new_n100), .c(new_n98), .o1(new_n146));
  aoi012aa1n06x5               g051(.a(new_n136), .b(new_n134), .c(new_n137), .o1(new_n147));
  oai013aa1d12x5               g052(.a(new_n147), .b(new_n143), .c(new_n146), .d(new_n134), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n145), .b(new_n149), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n16x5               g056(.a(\b[13] ), .b(\a[14] ), .o1(new_n152));
  nand02aa1d28x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  norb02aa1d21x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  nor042aa1n04x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand22aa1n04x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n155), .b(new_n150), .c(new_n156), .o1(new_n157));
  xnrc02aa1n02x5               g062(.a(new_n157), .b(new_n154), .out0(\s[14] ));
  nona23aa1n09x5               g063(.a(new_n153), .b(new_n156), .c(new_n155), .d(new_n152), .out0(new_n159));
  tech160nm_fiaoi012aa1n04x5   g064(.a(new_n152), .b(new_n155), .c(new_n153), .o1(new_n160));
  aoai13aa1n04x5               g065(.a(new_n160), .b(new_n159), .c(new_n145), .d(new_n149), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor022aa1n16x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nanp02aa1n06x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nor042aa1n06x5               g069(.a(\b[15] ), .b(\a[16] ), .o1(new_n165));
  nand42aa1n04x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  aoai13aa1n03x5               g072(.a(new_n167), .b(new_n163), .c(new_n161), .d(new_n164), .o1(new_n168));
  aoi112aa1n02x7               g073(.a(new_n163), .b(new_n167), .c(new_n161), .d(new_n164), .o1(new_n169));
  norb02aa1n02x7               g074(.a(new_n168), .b(new_n169), .out0(\s[16] ));
  xorc02aa1n02x5               g075(.a(\a[17] ), .b(\b[16] ), .out0(new_n171));
  nano22aa1n03x7               g076(.a(new_n136), .b(new_n135), .c(new_n137), .out0(new_n172));
  nona23aa1n09x5               g077(.a(new_n154), .b(new_n156), .c(new_n163), .d(new_n155), .out0(new_n173));
  nano22aa1n02x4               g078(.a(new_n165), .b(new_n164), .c(new_n166), .out0(new_n174));
  nano23aa1n06x5               g079(.a(new_n142), .b(new_n173), .c(new_n174), .d(new_n172), .out0(new_n175));
  aoai13aa1n12x5               g080(.a(new_n175), .b(new_n112), .c(new_n123), .d(new_n126), .o1(new_n176));
  nanb03aa1n03x5               g081(.a(new_n165), .b(new_n166), .c(new_n164), .out0(new_n177));
  nor043aa1n02x5               g082(.a(new_n159), .b(new_n163), .c(new_n177), .o1(new_n178));
  norp03aa1n06x5               g083(.a(new_n177), .b(new_n160), .c(new_n163), .o1(new_n179));
  tech160nm_fiao0012aa1n02p5x5 g084(.a(new_n165), .b(new_n163), .c(new_n166), .o(new_n180));
  aoi112aa1n09x5               g085(.a(new_n180), .b(new_n179), .c(new_n148), .d(new_n178), .o1(new_n181));
  xnbna2aa1n03x5               g086(.a(new_n171), .b(new_n176), .c(new_n181), .out0(\s[17] ));
  inv040aa1d32x5               g087(.a(\a[18] ), .o1(new_n183));
  norp02aa1n02x5               g088(.a(\b[16] ), .b(\a[17] ), .o1(new_n184));
  nanp02aa1n06x5               g089(.a(new_n176), .b(new_n181), .o1(new_n185));
  tech160nm_fiaoi012aa1n05x5   g090(.a(new_n184), .b(new_n185), .c(new_n171), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  inv000aa1d42x5               g092(.a(\a[17] ), .o1(new_n188));
  xroi22aa1d06x4               g093(.a(new_n188), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n189));
  inv000aa1d42x5               g094(.a(new_n189), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[17] ), .o1(new_n191));
  oao003aa1n02x5               g096(.a(new_n183), .b(new_n191), .c(new_n184), .carry(new_n192));
  inv000aa1d42x5               g097(.a(new_n192), .o1(new_n193));
  aoai13aa1n04x5               g098(.a(new_n193), .b(new_n190), .c(new_n176), .d(new_n181), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g100(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n02x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nand02aa1n06x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nor002aa1n06x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nand42aa1n06x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  aoai13aa1n03x5               g106(.a(new_n201), .b(new_n197), .c(new_n194), .d(new_n198), .o1(new_n202));
  aoi112aa1n02x5               g107(.a(new_n197), .b(new_n201), .c(new_n194), .d(new_n198), .o1(new_n203));
  norb02aa1n02x7               g108(.a(new_n202), .b(new_n203), .out0(\s[20] ));
  nanb03aa1n12x5               g109(.a(new_n199), .b(new_n200), .c(new_n198), .out0(new_n205));
  nona22aa1n03x5               g110(.a(new_n189), .b(new_n197), .c(new_n205), .out0(new_n206));
  oai022aa1d24x5               g111(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n207));
  oai122aa1n12x5               g112(.a(new_n207), .b(\a[19] ), .c(\b[18] ), .d(new_n183), .e(new_n191), .o1(new_n208));
  aoi012aa1n06x5               g113(.a(new_n199), .b(new_n197), .c(new_n200), .o1(new_n209));
  oai012aa1n12x5               g114(.a(new_n209), .b(new_n208), .c(new_n205), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  aoai13aa1n04x5               g116(.a(new_n211), .b(new_n206), .c(new_n176), .d(new_n181), .o1(new_n212));
  xorb03aa1n02x5               g117(.a(new_n212), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  xnrc02aa1n12x5               g119(.a(\b[20] ), .b(\a[21] ), .out0(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  xnrc02aa1n12x5               g121(.a(\b[21] ), .b(\a[22] ), .out0(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n03x5               g123(.a(new_n218), .b(new_n214), .c(new_n212), .d(new_n216), .o1(new_n219));
  aoi112aa1n02x5               g124(.a(new_n214), .b(new_n218), .c(new_n212), .d(new_n216), .o1(new_n220));
  norb02aa1n02x7               g125(.a(new_n219), .b(new_n220), .out0(\s[22] ));
  nor042aa1n06x5               g126(.a(new_n217), .b(new_n215), .o1(new_n222));
  nona23aa1n08x5               g127(.a(new_n189), .b(new_n222), .c(new_n205), .d(new_n197), .out0(new_n223));
  inv000aa1d42x5               g128(.a(\a[22] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\b[21] ), .o1(new_n225));
  oaoi03aa1n09x5               g130(.a(new_n224), .b(new_n225), .c(new_n214), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  aoi012aa1n02x5               g132(.a(new_n227), .b(new_n210), .c(new_n222), .o1(new_n228));
  aoai13aa1n04x5               g133(.a(new_n228), .b(new_n223), .c(new_n176), .d(new_n181), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g135(.a(\b[22] ), .b(\a[23] ), .o1(new_n231));
  tech160nm_fixorc02aa1n04x5   g136(.a(\a[23] ), .b(\b[22] ), .out0(new_n232));
  xorc02aa1n02x5               g137(.a(\a[24] ), .b(\b[23] ), .out0(new_n233));
  aoai13aa1n03x5               g138(.a(new_n233), .b(new_n231), .c(new_n229), .d(new_n232), .o1(new_n234));
  aoi112aa1n02x5               g139(.a(new_n231), .b(new_n233), .c(new_n229), .d(new_n232), .o1(new_n235));
  norb02aa1n02x7               g140(.a(new_n234), .b(new_n235), .out0(\s[24] ));
  nano22aa1n02x4               g141(.a(new_n199), .b(new_n198), .c(new_n200), .out0(new_n237));
  oai022aa1n02x5               g142(.a(new_n183), .b(new_n191), .c(\b[18] ), .d(\a[19] ), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n207), .b(new_n238), .out0(new_n239));
  inv000aa1n02x5               g144(.a(new_n209), .o1(new_n240));
  aoai13aa1n06x5               g145(.a(new_n222), .b(new_n240), .c(new_n239), .d(new_n237), .o1(new_n241));
  and002aa1n03x5               g146(.a(new_n233), .b(new_n232), .o(new_n242));
  inv000aa1n02x5               g147(.a(new_n242), .o1(new_n243));
  orn002aa1n02x5               g148(.a(\a[23] ), .b(\b[22] ), .o(new_n244));
  oao003aa1n02x5               g149(.a(\a[24] ), .b(\b[23] ), .c(new_n244), .carry(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n243), .c(new_n241), .d(new_n226), .o1(new_n246));
  nano32aa1n03x7               g151(.a(new_n206), .b(new_n233), .c(new_n222), .d(new_n232), .out0(new_n247));
  xorc02aa1n12x5               g152(.a(\a[25] ), .b(\b[24] ), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n246), .c(new_n185), .d(new_n247), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(new_n246), .b(new_n248), .c(new_n185), .d(new_n247), .o1(new_n250));
  norb02aa1n03x4               g155(.a(new_n249), .b(new_n250), .out0(\s[25] ));
  nor042aa1n03x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  inv000aa1n03x5               g157(.a(new_n252), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  aobi12aa1n02x7               g159(.a(new_n254), .b(new_n249), .c(new_n253), .out0(new_n255));
  nona22aa1n02x5               g160(.a(new_n249), .b(new_n254), .c(new_n252), .out0(new_n256));
  norb02aa1n03x4               g161(.a(new_n256), .b(new_n255), .out0(\s[26] ));
  inv000aa1n02x5               g162(.a(new_n112), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(new_n123), .b(new_n126), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(new_n259), .b(new_n258), .o1(new_n260));
  nanp02aa1n02x5               g165(.a(new_n148), .b(new_n178), .o1(new_n261));
  nona22aa1n02x4               g166(.a(new_n261), .b(new_n180), .c(new_n179), .out0(new_n262));
  and002aa1n06x5               g167(.a(new_n254), .b(new_n248), .o(new_n263));
  nano22aa1n03x7               g168(.a(new_n223), .b(new_n242), .c(new_n263), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n262), .c(new_n260), .d(new_n175), .o1(new_n265));
  oao003aa1n02x5               g170(.a(\a[26] ), .b(\b[25] ), .c(new_n253), .carry(new_n266));
  inv000aa1n02x5               g171(.a(new_n266), .o1(new_n267));
  aoi012aa1n06x5               g172(.a(new_n267), .b(new_n246), .c(new_n263), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[27] ), .b(\b[26] ), .out0(new_n269));
  xnbna2aa1n03x5               g174(.a(new_n269), .b(new_n268), .c(new_n265), .out0(\s[27] ));
  norp02aa1n02x5               g175(.a(\b[26] ), .b(\a[27] ), .o1(new_n271));
  inv040aa1n03x5               g176(.a(new_n271), .o1(new_n272));
  nand02aa1d04x5               g177(.a(new_n247), .b(new_n263), .o1(new_n273));
  aoi012aa1n06x5               g178(.a(new_n273), .b(new_n176), .c(new_n181), .o1(new_n274));
  aoai13aa1n04x5               g179(.a(new_n242), .b(new_n227), .c(new_n210), .d(new_n222), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n263), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n266), .b(new_n276), .c(new_n275), .d(new_n245), .o1(new_n277));
  oaih12aa1n02x5               g182(.a(new_n269), .b(new_n277), .c(new_n274), .o1(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[27] ), .b(\a[28] ), .out0(new_n279));
  tech160nm_fiaoi012aa1n02p5x5 g184(.a(new_n279), .b(new_n278), .c(new_n272), .o1(new_n280));
  aobi12aa1n02x7               g185(.a(new_n269), .b(new_n268), .c(new_n265), .out0(new_n281));
  nano22aa1n03x5               g186(.a(new_n281), .b(new_n272), .c(new_n279), .out0(new_n282));
  norp02aa1n03x5               g187(.a(new_n280), .b(new_n282), .o1(\s[28] ));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  norb02aa1n02x5               g189(.a(new_n269), .b(new_n279), .out0(new_n285));
  oaih12aa1n02x5               g190(.a(new_n285), .b(new_n277), .c(new_n274), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[28] ), .b(\b[27] ), .c(new_n272), .carry(new_n287));
  tech160nm_fiaoi012aa1n02p5x5 g192(.a(new_n284), .b(new_n286), .c(new_n287), .o1(new_n288));
  aobi12aa1n02x7               g193(.a(new_n285), .b(new_n268), .c(new_n265), .out0(new_n289));
  nano22aa1n03x5               g194(.a(new_n289), .b(new_n284), .c(new_n287), .out0(new_n290));
  norp02aa1n03x5               g195(.a(new_n288), .b(new_n290), .o1(\s[29] ));
  xorb03aa1n02x5               g196(.a(new_n114), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  norb03aa1n02x5               g198(.a(new_n269), .b(new_n284), .c(new_n279), .out0(new_n294));
  oaih12aa1n02x5               g199(.a(new_n294), .b(new_n277), .c(new_n274), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n287), .carry(new_n296));
  tech160nm_fiaoi012aa1n03p5x5 g201(.a(new_n293), .b(new_n295), .c(new_n296), .o1(new_n297));
  aobi12aa1n02x7               g202(.a(new_n294), .b(new_n268), .c(new_n265), .out0(new_n298));
  nano22aa1n03x5               g203(.a(new_n298), .b(new_n293), .c(new_n296), .out0(new_n299));
  norp02aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[30] ));
  norb02aa1n02x5               g205(.a(new_n294), .b(new_n293), .out0(new_n301));
  oaih12aa1n02x5               g206(.a(new_n301), .b(new_n277), .c(new_n274), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n304), .b(new_n302), .c(new_n303), .o1(new_n305));
  aobi12aa1n02x7               g210(.a(new_n301), .b(new_n268), .c(new_n265), .out0(new_n306));
  nano22aa1n03x5               g211(.a(new_n306), .b(new_n303), .c(new_n304), .out0(new_n307));
  norp02aa1n03x5               g212(.a(new_n305), .b(new_n307), .o1(\s[31] ));
  xnrb03aa1n02x5               g213(.a(new_n116), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g214(.a(\a[3] ), .b(\b[2] ), .c(new_n116), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n123), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n03x5               g217(.a(new_n124), .b(new_n123), .out0(new_n313));
  oai012aa1n06x5               g218(.a(new_n313), .b(\b[4] ), .c(\a[5] ), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g220(.a(new_n107), .b(new_n106), .out0(new_n316));
  aobi12aa1n06x5               g221(.a(new_n109), .b(new_n314), .c(new_n316), .out0(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n03x5               g223(.a(\a[7] ), .b(\b[6] ), .c(new_n317), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g225(.a(new_n127), .b(new_n259), .c(new_n258), .out0(\s[9] ));
endmodule


