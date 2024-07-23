// Benchmark "adder" written by ABC on Thu Jul 18 02:28:58 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n178, new_n180, new_n181, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n189, new_n190, new_n191,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n198, new_n199,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n208, new_n209, new_n210, new_n211, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n225, new_n226, new_n227, new_n228, new_n229, new_n230,
    new_n231, new_n232, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n245, new_n246,
    new_n247, new_n248, new_n249, new_n250, new_n251, new_n252, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n272, new_n273, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n281, new_n282, new_n283, new_n284, new_n285, new_n286,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n296,
    new_n298, new_n299, new_n301, new_n303, new_n305;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n24x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nanp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand42aa1n02x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nor042aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  tech160nm_fioai012aa1n04x5   g005(.a(new_n98), .b(new_n100), .c(new_n99), .o1(new_n101));
  nor002aa1n02x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanb02aa1n02x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\b[3] ), .o1(new_n105));
  nanb02aa1n02x5               g010(.a(\a[4] ), .b(new_n105), .out0(new_n106));
  nanp02aa1n02x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(new_n106), .b(new_n107), .o1(new_n108));
  norp03aa1n03x5               g013(.a(new_n101), .b(new_n104), .c(new_n108), .o1(new_n109));
  aob012aa1n02x5               g014(.a(new_n106), .b(new_n102), .c(new_n107), .out0(new_n110));
  nor022aa1n12x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor002aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n03x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  nor003aa1n02x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  tech160nm_fioai012aa1n05x5   g023(.a(new_n118), .b(new_n109), .c(new_n110), .o1(new_n119));
  inv000aa1d42x5               g024(.a(new_n111), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n113), .b(new_n112), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  oai022aa1n02x5               g027(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n123));
  nano22aa1n02x4               g028(.a(new_n115), .b(new_n122), .c(new_n123), .out0(new_n124));
  nano22aa1n03x7               g029(.a(new_n124), .b(new_n120), .c(new_n121), .out0(new_n125));
  tech160nm_fixnrc02aa1n02p5x5 g030(.a(\b[8] ), .b(\a[9] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n97), .b(new_n126), .c(new_n119), .d(new_n125), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  oaoi03aa1n12x5               g033(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n129));
  inv000aa1d42x5               g034(.a(new_n129), .o1(new_n130));
  nand22aa1n03x5               g035(.a(new_n119), .b(new_n125), .o1(new_n131));
  xnrc02aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .out0(new_n132));
  nona22aa1n02x4               g037(.a(new_n131), .b(new_n126), .c(new_n132), .out0(new_n133));
  nor042aa1n06x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  xobna2aa1n03x5               g041(.a(new_n136), .b(new_n133), .c(new_n130), .out0(\s[11] ));
  inv000aa1d42x5               g042(.a(new_n134), .o1(new_n138));
  aoai13aa1n02x5               g043(.a(new_n138), .b(new_n136), .c(new_n133), .d(new_n130), .o1(new_n139));
  xorb03aa1n02x5               g044(.a(new_n139), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nona23aa1n09x5               g047(.a(new_n142), .b(new_n135), .c(new_n134), .d(new_n141), .out0(new_n143));
  nona32aa1n02x4               g048(.a(new_n131), .b(new_n143), .c(new_n132), .d(new_n126), .out0(new_n144));
  oaoi03aa1n02x5               g049(.a(\a[12] ), .b(\b[11] ), .c(new_n138), .o1(new_n145));
  oabi12aa1n02x5               g050(.a(new_n145), .b(new_n130), .c(new_n143), .out0(new_n146));
  inv000aa1n02x5               g051(.a(new_n146), .o1(new_n147));
  tech160nm_fixnrc02aa1n02p5x5 g052(.a(\b[12] ), .b(\a[13] ), .out0(new_n148));
  xobna2aa1n03x5               g053(.a(new_n148), .b(new_n144), .c(new_n147), .out0(\s[13] ));
  orn002aa1n02x5               g054(.a(\a[13] ), .b(\b[12] ), .o(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n148), .c(new_n144), .d(new_n147), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g057(.a(\b[13] ), .b(\a[14] ), .out0(new_n153));
  nor043aa1n02x5               g058(.a(new_n153), .b(new_n148), .c(new_n126), .o1(new_n154));
  nona23aa1n03x5               g059(.a(new_n131), .b(new_n154), .c(new_n143), .d(new_n132), .out0(new_n155));
  nano23aa1n03x5               g060(.a(new_n134), .b(new_n141), .c(new_n142), .d(new_n135), .out0(new_n156));
  nor022aa1n02x5               g061(.a(new_n153), .b(new_n148), .o1(new_n157));
  aoai13aa1n06x5               g062(.a(new_n157), .b(new_n145), .c(new_n156), .d(new_n129), .o1(new_n158));
  oao003aa1n02x5               g063(.a(\a[14] ), .b(\b[13] ), .c(new_n150), .carry(new_n159));
  nanp03aa1n02x5               g064(.a(new_n155), .b(new_n158), .c(new_n159), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  tech160nm_fixorc02aa1n03p5x5 g067(.a(\a[15] ), .b(\b[14] ), .out0(new_n163));
  tech160nm_fixorc02aa1n04x5   g068(.a(\a[16] ), .b(\b[15] ), .out0(new_n164));
  aoi112aa1n02x5               g069(.a(new_n162), .b(new_n164), .c(new_n160), .d(new_n163), .o1(new_n165));
  aoai13aa1n03x5               g070(.a(new_n164), .b(new_n162), .c(new_n160), .d(new_n163), .o1(new_n166));
  norb02aa1n02x7               g071(.a(new_n166), .b(new_n165), .out0(\s[16] ));
  inv000aa1d42x5               g072(.a(\a[17] ), .o1(new_n168));
  and002aa1n06x5               g073(.a(new_n164), .b(new_n163), .o(new_n169));
  nona23aa1n09x5               g074(.a(new_n169), .b(new_n154), .c(new_n132), .d(new_n143), .out0(new_n170));
  aoi012aa1d18x5               g075(.a(new_n170), .b(new_n119), .c(new_n125), .o1(new_n171));
  inv000aa1d42x5               g076(.a(new_n169), .o1(new_n172));
  aoi112aa1n02x5               g077(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n173));
  oab012aa1n02x4               g078(.a(new_n173), .b(\a[16] ), .c(\b[15] ), .out0(new_n174));
  aoai13aa1n12x5               g079(.a(new_n174), .b(new_n172), .c(new_n158), .d(new_n159), .o1(new_n175));
  nor042aa1n04x5               g080(.a(new_n175), .b(new_n171), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[16] ), .c(new_n168), .out0(\s[17] ));
  oaoi03aa1n03x5               g082(.a(\a[17] ), .b(\b[16] ), .c(new_n176), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g084(.a(\a[19] ), .o1(new_n180));
  inv000aa1d42x5               g085(.a(\a[18] ), .o1(new_n181));
  xroi22aa1d06x4               g086(.a(new_n168), .b(\b[16] ), .c(new_n181), .d(\b[17] ), .out0(new_n182));
  inv000aa1d42x5               g087(.a(\b[17] ), .o1(new_n183));
  oai022aa1d18x5               g088(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n184));
  oa0012aa1n02x5               g089(.a(new_n184), .b(new_n183), .c(new_n181), .o(new_n185));
  oaoi13aa1n03x5               g090(.a(new_n185), .b(new_n182), .c(new_n175), .d(new_n171), .o1(new_n186));
  xorb03aa1n03x5               g091(.a(new_n186), .b(\b[18] ), .c(new_n180), .out0(\s[19] ));
  xnrc02aa1n02x5               g092(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g093(.a(\b[18] ), .o1(new_n189));
  nanp02aa1n02x5               g094(.a(new_n189), .b(new_n180), .o1(new_n190));
  oai012aa1n03x5               g095(.a(new_n182), .b(new_n175), .c(new_n171), .o1(new_n191));
  xnrc02aa1n02x5               g096(.a(\b[18] ), .b(\a[19] ), .out0(new_n192));
  aoib12aa1n02x7               g097(.a(new_n192), .b(new_n191), .c(new_n185), .out0(new_n193));
  xnrc02aa1n02x5               g098(.a(\b[19] ), .b(\a[20] ), .out0(new_n194));
  nano22aa1n02x4               g099(.a(new_n193), .b(new_n190), .c(new_n194), .out0(new_n195));
  oaoi13aa1n03x5               g100(.a(new_n194), .b(new_n190), .c(new_n186), .d(new_n192), .o1(new_n196));
  norp02aa1n03x5               g101(.a(new_n196), .b(new_n195), .o1(\s[20] ));
  norb03aa1n03x5               g102(.a(new_n182), .b(new_n194), .c(new_n192), .out0(new_n198));
  tech160nm_fioai012aa1n05x5   g103(.a(new_n198), .b(new_n175), .c(new_n171), .o1(new_n199));
  inv000aa1d42x5               g104(.a(\b[19] ), .o1(new_n200));
  oai122aa1n02x7               g105(.a(new_n184), .b(new_n180), .c(new_n189), .d(new_n181), .e(new_n183), .o1(new_n201));
  oai112aa1n06x5               g106(.a(new_n201), .b(new_n190), .c(\b[19] ), .d(\a[20] ), .o1(new_n202));
  oaib12aa1n18x5               g107(.a(new_n202), .b(new_n200), .c(\a[20] ), .out0(new_n203));
  nor042aa1n06x5               g108(.a(\b[20] ), .b(\a[21] ), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(\b[20] ), .b(\a[21] ), .o1(new_n205));
  nanb02aa1n02x5               g110(.a(new_n204), .b(new_n205), .out0(new_n206));
  xobna2aa1n03x5               g111(.a(new_n206), .b(new_n199), .c(new_n203), .out0(\s[21] ));
  inv030aa1n03x5               g112(.a(new_n204), .o1(new_n208));
  aoi012aa1n06x5               g113(.a(new_n206), .b(new_n199), .c(new_n203), .o1(new_n209));
  norp02aa1n02x5               g114(.a(\b[21] ), .b(\a[22] ), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(\b[21] ), .b(\a[22] ), .o1(new_n211));
  nanb02aa1n02x5               g116(.a(new_n210), .b(new_n211), .out0(new_n212));
  nano22aa1n02x4               g117(.a(new_n209), .b(new_n208), .c(new_n212), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n203), .o1(new_n214));
  oaoi13aa1n02x5               g119(.a(new_n214), .b(new_n198), .c(new_n175), .d(new_n171), .o1(new_n215));
  oaoi13aa1n02x7               g120(.a(new_n212), .b(new_n208), .c(new_n215), .d(new_n206), .o1(new_n216));
  norp02aa1n03x5               g121(.a(new_n216), .b(new_n213), .o1(\s[22] ));
  inv000aa1d42x5               g122(.a(\a[23] ), .o1(new_n218));
  nona23aa1n06x5               g123(.a(new_n211), .b(new_n205), .c(new_n204), .d(new_n210), .out0(new_n219));
  norb02aa1n02x5               g124(.a(new_n198), .b(new_n219), .out0(new_n220));
  oaoi03aa1n02x5               g125(.a(\a[22] ), .b(\b[21] ), .c(new_n208), .o1(new_n221));
  oabi12aa1n02x5               g126(.a(new_n221), .b(new_n203), .c(new_n219), .out0(new_n222));
  oaoi13aa1n06x5               g127(.a(new_n222), .b(new_n220), .c(new_n175), .d(new_n171), .o1(new_n223));
  xorb03aa1n02x5               g128(.a(new_n223), .b(\b[22] ), .c(new_n218), .out0(\s[23] ));
  and002aa1n02x5               g129(.a(\b[22] ), .b(\a[23] ), .o(new_n225));
  xorc02aa1n02x5               g130(.a(\a[23] ), .b(\b[22] ), .out0(new_n226));
  nanp02aa1n02x5               g131(.a(new_n223), .b(new_n226), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(\b[23] ), .b(\a[24] ), .o1(new_n228));
  norp02aa1n02x5               g133(.a(\b[23] ), .b(\a[24] ), .o1(new_n229));
  norb02aa1n03x4               g134(.a(new_n228), .b(new_n229), .out0(new_n230));
  nona22aa1n02x4               g135(.a(new_n227), .b(new_n230), .c(new_n225), .out0(new_n231));
  aoai13aa1n02x5               g136(.a(new_n230), .b(new_n225), .c(new_n223), .d(new_n226), .o1(new_n232));
  nanp02aa1n02x5               g137(.a(new_n231), .b(new_n232), .o1(\s[24] ));
  nano22aa1n02x4               g138(.a(new_n219), .b(new_n226), .c(new_n230), .out0(new_n234));
  nano23aa1n02x4               g139(.a(new_n194), .b(new_n192), .c(new_n234), .d(new_n182), .out0(new_n235));
  and002aa1n02x5               g140(.a(\b[19] ), .b(\a[20] ), .o(new_n236));
  nanb02aa1n02x5               g141(.a(\b[22] ), .b(new_n218), .out0(new_n237));
  nano23aa1n02x4               g142(.a(new_n229), .b(new_n225), .c(new_n237), .d(new_n228), .out0(new_n238));
  nona23aa1n02x4               g143(.a(new_n202), .b(new_n238), .c(new_n219), .d(new_n236), .out0(new_n239));
  aoi112aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n240));
  aoi113aa1n03x7               g145(.a(new_n229), .b(new_n240), .c(new_n221), .d(new_n226), .e(new_n228), .o1(new_n241));
  nanp02aa1n02x5               g146(.a(new_n239), .b(new_n241), .o1(new_n242));
  oaoi13aa1n03x5               g147(.a(new_n242), .b(new_n235), .c(new_n175), .d(new_n171), .o1(new_n243));
  xnrb03aa1n03x5               g148(.a(new_n243), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g149(.a(\b[24] ), .b(\a[25] ), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  oai012aa1n03x5               g151(.a(new_n235), .b(new_n175), .c(new_n171), .o1(new_n247));
  xnrc02aa1n02x5               g152(.a(\b[24] ), .b(\a[25] ), .out0(new_n248));
  aoib12aa1n02x7               g153(.a(new_n248), .b(new_n247), .c(new_n242), .out0(new_n249));
  xnrc02aa1n02x5               g154(.a(\b[25] ), .b(\a[26] ), .out0(new_n250));
  nano22aa1n02x4               g155(.a(new_n249), .b(new_n246), .c(new_n250), .out0(new_n251));
  oaoi13aa1n03x5               g156(.a(new_n250), .b(new_n246), .c(new_n243), .d(new_n248), .o1(new_n252));
  norp02aa1n03x5               g157(.a(new_n252), .b(new_n251), .o1(\s[26] ));
  nor042aa1n02x5               g158(.a(new_n250), .b(new_n248), .o1(new_n254));
  and003aa1n03x5               g159(.a(new_n198), .b(new_n234), .c(new_n254), .o(new_n255));
  tech160nm_fioai012aa1n05x5   g160(.a(new_n255), .b(new_n175), .c(new_n171), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n254), .o1(new_n257));
  oao003aa1n02x5               g162(.a(\a[26] ), .b(\b[25] ), .c(new_n246), .carry(new_n258));
  aoai13aa1n12x5               g163(.a(new_n258), .b(new_n257), .c(new_n239), .d(new_n241), .o1(new_n259));
  inv000aa1n02x5               g164(.a(new_n259), .o1(new_n260));
  tech160nm_fixnrc02aa1n05x5   g165(.a(\b[26] ), .b(\a[27] ), .out0(new_n261));
  xobna2aa1n03x5               g166(.a(new_n261), .b(new_n256), .c(new_n260), .out0(\s[27] ));
  norp02aa1n02x5               g167(.a(\b[26] ), .b(\a[27] ), .o1(new_n263));
  inv040aa1n03x5               g168(.a(new_n263), .o1(new_n264));
  tech160nm_fixnrc02aa1n05x5   g169(.a(\b[27] ), .b(\a[28] ), .out0(new_n265));
  aoi022aa1n02x7               g170(.a(new_n256), .b(new_n260), .c(\b[26] ), .d(\a[27] ), .o1(new_n266));
  nano22aa1n03x5               g171(.a(new_n266), .b(new_n264), .c(new_n265), .out0(new_n267));
  oaoi13aa1n04x5               g172(.a(new_n259), .b(new_n255), .c(new_n175), .d(new_n171), .o1(new_n268));
  and002aa1n02x5               g173(.a(\b[26] ), .b(\a[27] ), .o(new_n269));
  oaoi13aa1n02x7               g174(.a(new_n265), .b(new_n264), .c(new_n268), .d(new_n269), .o1(new_n270));
  norp02aa1n03x5               g175(.a(new_n270), .b(new_n267), .o1(\s[28] ));
  nor002aa1n02x5               g176(.a(new_n265), .b(new_n261), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  oao003aa1n02x5               g178(.a(\a[28] ), .b(\b[27] ), .c(new_n264), .carry(new_n274));
  xnrc02aa1n02x5               g179(.a(\b[28] ), .b(\a[29] ), .out0(new_n275));
  oaoi13aa1n02x7               g180(.a(new_n275), .b(new_n274), .c(new_n268), .d(new_n273), .o1(new_n276));
  tech160nm_fiaoi012aa1n02p5x5 g181(.a(new_n273), .b(new_n256), .c(new_n260), .o1(new_n277));
  nano22aa1n03x5               g182(.a(new_n277), .b(new_n274), .c(new_n275), .out0(new_n278));
  norp02aa1n03x5               g183(.a(new_n276), .b(new_n278), .o1(\s[29] ));
  xorb03aa1n02x5               g184(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb02aa1n06x5               g185(.a(new_n275), .b(new_n272), .out0(new_n281));
  oao003aa1n02x5               g186(.a(\a[29] ), .b(\b[28] ), .c(new_n274), .carry(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[29] ), .b(\a[30] ), .out0(new_n283));
  oaoi13aa1n02x7               g188(.a(new_n283), .b(new_n282), .c(new_n268), .d(new_n281), .o1(new_n284));
  tech160nm_fiaoi012aa1n02p5x5 g189(.a(new_n281), .b(new_n256), .c(new_n260), .o1(new_n285));
  nano22aa1n03x5               g190(.a(new_n285), .b(new_n282), .c(new_n283), .out0(new_n286));
  norp02aa1n03x5               g191(.a(new_n284), .b(new_n286), .o1(\s[30] ));
  xnrc02aa1n02x5               g192(.a(\b[30] ), .b(\a[31] ), .out0(new_n288));
  nona22aa1n02x4               g193(.a(new_n272), .b(new_n275), .c(new_n283), .out0(new_n289));
  oao003aa1n02x5               g194(.a(\a[30] ), .b(\b[29] ), .c(new_n282), .carry(new_n290));
  oaoi13aa1n02x7               g195(.a(new_n288), .b(new_n290), .c(new_n268), .d(new_n289), .o1(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n289), .b(new_n256), .c(new_n260), .o1(new_n292));
  nano22aa1n03x5               g197(.a(new_n292), .b(new_n288), .c(new_n290), .out0(new_n293));
  norp02aa1n03x5               g198(.a(new_n291), .b(new_n293), .o1(\s[31] ));
  xnrb03aa1n02x5               g199(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g200(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n296));
  xorb03aa1n02x5               g201(.a(new_n296), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  oai012aa1n02x5               g202(.a(new_n117), .b(new_n109), .c(new_n110), .o1(new_n298));
  orn003aa1n02x5               g203(.a(new_n109), .b(new_n117), .c(new_n110), .o(new_n299));
  nanp02aa1n02x5               g204(.a(new_n299), .b(new_n298), .o1(\s[5] ));
  aob012aa1n02x5               g205(.a(new_n299), .b(\b[4] ), .c(\a[5] ), .out0(new_n301));
  xnrb03aa1n02x5               g206(.a(new_n301), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g207(.a(\a[6] ), .b(\b[5] ), .c(new_n301), .o1(new_n303));
  xorb03aa1n02x5               g208(.a(new_n303), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g209(.a(new_n113), .b(new_n303), .c(new_n114), .o1(new_n305));
  xnbna2aa1n03x5               g210(.a(new_n305), .b(new_n120), .c(new_n112), .out0(\s[8] ));
  xobna2aa1n03x5               g211(.a(new_n126), .b(new_n119), .c(new_n125), .out0(\s[9] ));
endmodule


