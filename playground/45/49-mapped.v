// Benchmark "adder" written by ABC on Thu Jul 18 11:30:44 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n148, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n159, new_n160, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n178, new_n179, new_n180, new_n181,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n211, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n227, new_n228, new_n229, new_n230,
    new_n231, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n307, new_n310, new_n311, new_n313,
    new_n314, new_n315;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_fixorc02aa1n03p5x5 g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  nor022aa1n16x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(new_n98), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\a[2] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[1] ), .o1(new_n101));
  nanp02aa1n04x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  tech160nm_fioaoi03aa1n03p5x5 g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  nor022aa1n08x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor022aa1n16x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n06x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  oai012aa1n02x7               g013(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n109));
  oai012aa1n12x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .o1(new_n110));
  norp02aa1n04x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  tech160nm_finand02aa1n03p5x5 g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nor022aa1n06x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nand42aa1n03x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  nand42aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nor002aa1n04x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nanb02aa1n06x5               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  tech160nm_fixorc02aa1n05x5   g023(.a(\a[8] ), .b(\b[7] ), .out0(new_n119));
  norb03aa1n09x5               g024(.a(new_n119), .b(new_n115), .c(new_n118), .out0(new_n120));
  oaoi13aa1n04x5               g025(.a(new_n111), .b(new_n116), .c(new_n113), .d(new_n117), .o1(new_n121));
  aob012aa1n02x5               g026(.a(new_n112), .b(\b[7] ), .c(\a[8] ), .out0(new_n122));
  oai022aa1n06x5               g027(.a(new_n121), .b(new_n122), .c(\b[7] ), .d(\a[8] ), .o1(new_n123));
  tech160nm_fixorc02aa1n03p5x5 g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n06x5               g029(.a(new_n124), .b(new_n123), .c(new_n120), .d(new_n110), .o1(new_n125));
  xnbna2aa1n03x5               g030(.a(new_n97), .b(new_n125), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g031(.a(\a[10] ), .o1(new_n127));
  inv000aa1d42x5               g032(.a(\b[9] ), .o1(new_n128));
  tech160nm_fiaoi012aa1n05x5   g033(.a(new_n98), .b(new_n127), .c(new_n128), .o1(new_n129));
  aoi022aa1n02x5               g034(.a(new_n125), .b(new_n129), .c(\b[9] ), .d(\a[10] ), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n04x5               g036(.a(\b[11] ), .b(\a[12] ), .o1(new_n132));
  inv000aa1d42x5               g037(.a(new_n132), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  nand42aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norp02aa1n04x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  aoi012aa1n02x5               g041(.a(new_n136), .b(new_n130), .c(new_n135), .o1(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n133), .c(new_n134), .out0(\s[12] ));
  nano23aa1n06x5               g043(.a(new_n132), .b(new_n136), .c(new_n134), .d(new_n135), .out0(new_n139));
  and003aa1n02x5               g044(.a(new_n139), .b(new_n124), .c(new_n97), .o(new_n140));
  aoai13aa1n06x5               g045(.a(new_n140), .b(new_n123), .c(new_n120), .d(new_n110), .o1(new_n141));
  inv000aa1n02x5               g046(.a(new_n136), .o1(new_n142));
  aoai13aa1n06x5               g047(.a(new_n142), .b(new_n129), .c(\b[9] ), .d(\a[10] ), .o1(new_n143));
  nanp02aa1n02x5               g048(.a(new_n134), .b(new_n135), .o1(new_n144));
  aoib12aa1n02x7               g049(.a(new_n132), .b(new_n143), .c(new_n144), .out0(new_n145));
  xorc02aa1n06x5               g050(.a(\a[13] ), .b(\b[12] ), .out0(new_n146));
  xnbna2aa1n03x5               g051(.a(new_n146), .b(new_n141), .c(new_n145), .out0(\s[13] ));
  inv000aa1d42x5               g052(.a(\a[14] ), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n141), .b(new_n145), .o1(new_n149));
  nor042aa1n03x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n150), .b(new_n149), .c(new_n146), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(new_n148), .out0(\s[14] ));
  xorc02aa1n02x5               g057(.a(\a[14] ), .b(\b[13] ), .out0(new_n153));
  nanp02aa1n02x5               g058(.a(new_n153), .b(new_n146), .o1(new_n154));
  inv000aa1d42x5               g059(.a(\b[13] ), .o1(new_n155));
  tech160nm_fioaoi03aa1n02p5x5 g060(.a(new_n148), .b(new_n155), .c(new_n150), .o1(new_n156));
  aoai13aa1n04x5               g061(.a(new_n156), .b(new_n154), .c(new_n141), .d(new_n145), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n03x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  nand42aa1n16x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nor002aa1n02x5               g065(.a(\b[15] ), .b(\a[16] ), .o1(new_n161));
  nanp02aa1n12x5               g066(.a(\b[15] ), .b(\a[16] ), .o1(new_n162));
  nanb02aa1n02x5               g067(.a(new_n161), .b(new_n162), .out0(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n159), .c(new_n157), .d(new_n160), .o1(new_n164));
  aoi112aa1n03x5               g069(.a(new_n159), .b(new_n163), .c(new_n157), .d(new_n160), .o1(new_n165));
  nanb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(\s[16] ));
  nano23aa1n03x5               g071(.a(new_n159), .b(new_n161), .c(new_n162), .d(new_n160), .out0(new_n167));
  nand23aa1n03x5               g072(.a(new_n167), .b(new_n146), .c(new_n153), .o1(new_n168));
  nano32aa1n03x7               g073(.a(new_n168), .b(new_n139), .c(new_n124), .d(new_n97), .out0(new_n169));
  aoai13aa1n12x5               g074(.a(new_n169), .b(new_n123), .c(new_n110), .d(new_n120), .o1(new_n170));
  oaoi03aa1n02x5               g075(.a(new_n127), .b(new_n128), .c(new_n98), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n133), .b(new_n144), .c(new_n171), .d(new_n142), .o1(new_n172));
  norp02aa1n02x5               g077(.a(new_n161), .b(new_n159), .o1(new_n173));
  aoai13aa1n03x5               g078(.a(new_n173), .b(new_n156), .c(\a[15] ), .d(\b[14] ), .o1(new_n174));
  aboi22aa1n09x5               g079(.a(new_n168), .b(new_n172), .c(new_n174), .d(new_n162), .out0(new_n175));
  nand02aa1d06x5               g080(.a(new_n170), .b(new_n175), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g082(.a(\a[18] ), .o1(new_n178));
  inv040aa1d32x5               g083(.a(\a[17] ), .o1(new_n179));
  inv030aa1d32x5               g084(.a(\b[16] ), .o1(new_n180));
  oaoi03aa1n03x5               g085(.a(new_n179), .b(new_n180), .c(new_n176), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[17] ), .c(new_n178), .out0(\s[18] ));
  xroi22aa1d06x4               g087(.a(new_n179), .b(\b[16] ), .c(new_n178), .d(\b[17] ), .out0(new_n183));
  nanp02aa1n02x5               g088(.a(new_n180), .b(new_n179), .o1(new_n184));
  oaoi03aa1n09x5               g089(.a(\a[18] ), .b(\b[17] ), .c(new_n184), .o1(new_n185));
  nor002aa1n12x5               g090(.a(\b[18] ), .b(\a[19] ), .o1(new_n186));
  nand22aa1n09x5               g091(.a(\b[18] ), .b(\a[19] ), .o1(new_n187));
  nanb02aa1n02x5               g092(.a(new_n186), .b(new_n187), .out0(new_n188));
  inv000aa1d42x5               g093(.a(new_n188), .o1(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n185), .c(new_n176), .d(new_n183), .o1(new_n190));
  aoi112aa1n02x5               g095(.a(new_n189), .b(new_n185), .c(new_n176), .d(new_n183), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n190), .b(new_n191), .out0(\s[19] ));
  xnrc02aa1n02x5               g097(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  nand22aa1n12x5               g099(.a(\b[19] ), .b(\a[20] ), .o1(new_n195));
  nanb02aa1n02x5               g100(.a(new_n194), .b(new_n195), .out0(new_n196));
  tech160nm_fioai012aa1n03p5x5 g101(.a(new_n190), .b(\b[18] ), .c(\a[19] ), .o1(new_n197));
  nand02aa1n02x5               g102(.a(new_n197), .b(new_n196), .o1(new_n198));
  nona22aa1n02x5               g103(.a(new_n190), .b(new_n196), .c(new_n186), .out0(new_n199));
  nanp02aa1n03x5               g104(.a(new_n198), .b(new_n199), .o1(\s[20] ));
  nano23aa1n06x5               g105(.a(new_n186), .b(new_n194), .c(new_n195), .d(new_n187), .out0(new_n201));
  nanp02aa1n02x5               g106(.a(new_n183), .b(new_n201), .o1(new_n202));
  oai022aa1n02x7               g107(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n203));
  oaib12aa1n06x5               g108(.a(new_n203), .b(new_n178), .c(\b[17] ), .out0(new_n204));
  nona23aa1n09x5               g109(.a(new_n195), .b(new_n187), .c(new_n186), .d(new_n194), .out0(new_n205));
  aoi012aa1n09x5               g110(.a(new_n194), .b(new_n186), .c(new_n195), .o1(new_n206));
  oai012aa1d24x5               g111(.a(new_n206), .b(new_n205), .c(new_n204), .o1(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n202), .c(new_n170), .d(new_n175), .o1(new_n209));
  xorb03aa1n02x5               g114(.a(new_n209), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g115(.a(\b[20] ), .b(\a[21] ), .o1(new_n211));
  xorc02aa1n02x5               g116(.a(\a[21] ), .b(\b[20] ), .out0(new_n212));
  xorc02aa1n02x5               g117(.a(\a[22] ), .b(\b[21] ), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoai13aa1n02x5               g119(.a(new_n214), .b(new_n211), .c(new_n209), .d(new_n212), .o1(new_n215));
  aoi112aa1n03x5               g120(.a(new_n211), .b(new_n214), .c(new_n209), .d(new_n212), .o1(new_n216));
  nanb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(\s[22] ));
  inv000aa1d42x5               g122(.a(\a[21] ), .o1(new_n218));
  inv040aa1d32x5               g123(.a(\a[22] ), .o1(new_n219));
  xroi22aa1d06x4               g124(.a(new_n218), .b(\b[20] ), .c(new_n219), .d(\b[21] ), .out0(new_n220));
  nanp03aa1n02x5               g125(.a(new_n220), .b(new_n183), .c(new_n201), .o1(new_n221));
  inv000aa1d42x5               g126(.a(\b[21] ), .o1(new_n222));
  oao003aa1n06x5               g127(.a(new_n219), .b(new_n222), .c(new_n211), .carry(new_n223));
  aoi012aa1n02x5               g128(.a(new_n223), .b(new_n207), .c(new_n220), .o1(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n221), .c(new_n170), .d(new_n175), .o1(new_n225));
  xorb03aa1n02x5               g130(.a(new_n225), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g131(.a(\b[22] ), .b(\a[23] ), .o1(new_n227));
  xorc02aa1n12x5               g132(.a(\a[23] ), .b(\b[22] ), .out0(new_n228));
  xnrc02aa1n12x5               g133(.a(\b[23] ), .b(\a[24] ), .out0(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n227), .c(new_n225), .d(new_n228), .o1(new_n230));
  aoi112aa1n03x5               g135(.a(new_n227), .b(new_n229), .c(new_n225), .d(new_n228), .o1(new_n231));
  nanb02aa1n03x5               g136(.a(new_n231), .b(new_n230), .out0(\s[24] ));
  nand42aa1n02x5               g137(.a(new_n120), .b(new_n110), .o1(new_n233));
  oa0022aa1n02x5               g138(.a(new_n121), .b(new_n122), .c(\a[8] ), .d(\b[7] ), .o(new_n234));
  nanp02aa1n04x5               g139(.a(new_n233), .b(new_n234), .o1(new_n235));
  nanp02aa1n02x5               g140(.a(new_n174), .b(new_n162), .o1(new_n236));
  oai012aa1n02x7               g141(.a(new_n236), .b(new_n145), .c(new_n168), .o1(new_n237));
  norb02aa1n03x5               g142(.a(new_n228), .b(new_n229), .out0(new_n238));
  nano22aa1n02x4               g143(.a(new_n202), .b(new_n238), .c(new_n220), .out0(new_n239));
  aoai13aa1n02x5               g144(.a(new_n239), .b(new_n237), .c(new_n235), .d(new_n169), .o1(new_n240));
  inv000aa1n04x5               g145(.a(new_n206), .o1(new_n241));
  aoai13aa1n06x5               g146(.a(new_n220), .b(new_n241), .c(new_n201), .d(new_n185), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n223), .o1(new_n243));
  inv000aa1n03x5               g148(.a(new_n238), .o1(new_n244));
  oai022aa1n02x5               g149(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n245));
  aob012aa1n02x5               g150(.a(new_n245), .b(\b[23] ), .c(\a[24] ), .out0(new_n246));
  aoai13aa1n12x5               g151(.a(new_n246), .b(new_n244), .c(new_n242), .d(new_n243), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  xorc02aa1n12x5               g153(.a(\a[25] ), .b(\b[24] ), .out0(new_n249));
  xnbna2aa1n03x5               g154(.a(new_n249), .b(new_n240), .c(new_n248), .out0(\s[25] ));
  nanp02aa1n02x5               g155(.a(new_n240), .b(new_n248), .o1(new_n251));
  norp02aa1n02x5               g156(.a(\b[24] ), .b(\a[25] ), .o1(new_n252));
  nor002aa1n02x5               g157(.a(\b[25] ), .b(\a[26] ), .o1(new_n253));
  nand42aa1n03x5               g158(.a(\b[25] ), .b(\a[26] ), .o1(new_n254));
  norb02aa1n06x4               g159(.a(new_n254), .b(new_n253), .out0(new_n255));
  inv040aa1n03x5               g160(.a(new_n255), .o1(new_n256));
  aoai13aa1n03x5               g161(.a(new_n256), .b(new_n252), .c(new_n251), .d(new_n249), .o1(new_n257));
  aoai13aa1n02x5               g162(.a(new_n249), .b(new_n247), .c(new_n176), .d(new_n239), .o1(new_n258));
  nona22aa1n03x5               g163(.a(new_n258), .b(new_n256), .c(new_n252), .out0(new_n259));
  nanp02aa1n02x5               g164(.a(new_n257), .b(new_n259), .o1(\s[26] ));
  norb02aa1n09x5               g165(.a(new_n249), .b(new_n256), .out0(new_n261));
  nano22aa1n06x5               g166(.a(new_n221), .b(new_n238), .c(new_n261), .out0(new_n262));
  aoai13aa1n06x5               g167(.a(new_n262), .b(new_n237), .c(new_n235), .d(new_n169), .o1(new_n263));
  oai012aa1n02x5               g168(.a(new_n254), .b(new_n253), .c(new_n252), .o1(new_n264));
  aobi12aa1n12x5               g169(.a(new_n264), .b(new_n247), .c(new_n261), .out0(new_n265));
  xorc02aa1n02x5               g170(.a(\a[27] ), .b(\b[26] ), .out0(new_n266));
  xnbna2aa1n03x5               g171(.a(new_n266), .b(new_n265), .c(new_n263), .out0(\s[27] ));
  nanp02aa1n06x5               g172(.a(new_n265), .b(new_n263), .o1(new_n268));
  norp02aa1n02x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  tech160nm_fixorc02aa1n02p5x5 g174(.a(\a[28] ), .b(\b[27] ), .out0(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n269), .c(new_n268), .d(new_n266), .o1(new_n272));
  aobi12aa1n06x5               g177(.a(new_n262), .b(new_n170), .c(new_n175), .out0(new_n273));
  aoai13aa1n03x5               g178(.a(new_n238), .b(new_n223), .c(new_n207), .d(new_n220), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n261), .o1(new_n275));
  aoai13aa1n02x7               g180(.a(new_n264), .b(new_n275), .c(new_n274), .d(new_n246), .o1(new_n276));
  oaih12aa1n02x5               g181(.a(new_n266), .b(new_n276), .c(new_n273), .o1(new_n277));
  nona22aa1n02x5               g182(.a(new_n277), .b(new_n271), .c(new_n269), .out0(new_n278));
  nanp02aa1n03x5               g183(.a(new_n272), .b(new_n278), .o1(\s[28] ));
  xnrc02aa1n02x5               g184(.a(\b[28] ), .b(\a[29] ), .out0(new_n280));
  and002aa1n02x5               g185(.a(new_n270), .b(new_n266), .o(new_n281));
  oaih12aa1n02x5               g186(.a(new_n281), .b(new_n276), .c(new_n273), .o1(new_n282));
  aoi112aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n283));
  oab012aa1n02x4               g188(.a(new_n283), .b(\a[28] ), .c(\b[27] ), .out0(new_n284));
  tech160nm_fiaoi012aa1n02p5x5 g189(.a(new_n280), .b(new_n282), .c(new_n284), .o1(new_n285));
  aobi12aa1n02x7               g190(.a(new_n281), .b(new_n265), .c(new_n263), .out0(new_n286));
  nano22aa1n03x5               g191(.a(new_n286), .b(new_n280), .c(new_n284), .out0(new_n287));
  norp02aa1n03x5               g192(.a(new_n285), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g193(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g194(.a(\b[29] ), .b(\a[30] ), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n280), .b(new_n266), .c(new_n270), .out0(new_n291));
  oaih12aa1n02x5               g196(.a(new_n291), .b(new_n276), .c(new_n273), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n284), .carry(new_n293));
  tech160nm_fiaoi012aa1n02p5x5 g198(.a(new_n290), .b(new_n292), .c(new_n293), .o1(new_n294));
  aobi12aa1n02x7               g199(.a(new_n291), .b(new_n265), .c(new_n263), .out0(new_n295));
  nano22aa1n03x5               g200(.a(new_n295), .b(new_n290), .c(new_n293), .out0(new_n296));
  norp02aa1n03x5               g201(.a(new_n294), .b(new_n296), .o1(\s[30] ));
  nano23aa1n06x5               g202(.a(new_n290), .b(new_n280), .c(new_n270), .d(new_n266), .out0(new_n298));
  oaih12aa1n02x5               g203(.a(new_n298), .b(new_n276), .c(new_n273), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[30] ), .b(\b[29] ), .c(new_n293), .carry(new_n300));
  xnrc02aa1n02x5               g205(.a(\b[30] ), .b(\a[31] ), .out0(new_n301));
  tech160nm_fiaoi012aa1n02p5x5 g206(.a(new_n301), .b(new_n299), .c(new_n300), .o1(new_n302));
  aobi12aa1n03x5               g207(.a(new_n298), .b(new_n265), .c(new_n263), .out0(new_n303));
  nano22aa1n03x5               g208(.a(new_n303), .b(new_n300), .c(new_n301), .out0(new_n304));
  norp02aa1n03x5               g209(.a(new_n302), .b(new_n304), .o1(\s[31] ));
  xnrb03aa1n02x5               g210(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g211(.a(\a[3] ), .b(\b[2] ), .c(new_n103), .o1(new_n307));
  xorb03aa1n02x5               g212(.a(new_n307), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g213(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoai13aa1n02x5               g214(.a(new_n118), .b(new_n113), .c(new_n110), .d(new_n114), .o1(new_n310));
  aoi112aa1n03x5               g215(.a(new_n113), .b(new_n118), .c(new_n110), .d(new_n114), .o1(new_n311));
  nanb02aa1n02x5               g216(.a(new_n311), .b(new_n310), .out0(\s[6] ));
  norb02aa1n02x5               g217(.a(new_n112), .b(new_n111), .out0(new_n313));
  aoai13aa1n06x5               g218(.a(new_n313), .b(new_n311), .c(\b[5] ), .d(\a[6] ), .o1(new_n314));
  nona22aa1n02x4               g219(.a(new_n116), .b(new_n311), .c(new_n313), .out0(new_n315));
  nanp02aa1n02x5               g220(.a(new_n315), .b(new_n314), .o1(\s[7] ));
  xobna2aa1n03x5               g221(.a(new_n119), .b(new_n314), .c(new_n112), .out0(\s[8] ));
  xnbna2aa1n03x5               g222(.a(new_n124), .b(new_n233), .c(new_n234), .out0(\s[9] ));
endmodule

